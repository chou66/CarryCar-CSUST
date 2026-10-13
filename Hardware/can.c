#include "can.h"

#define FDCAN_TX_TIMEOUT_MS 20U

typedef struct
{
    volatile uint8_t valid;
    volatile uint8_t status;
    volatile uint32_t tick_ms;
} MotorStatusCache_t;

volatile uint32_t can_error_step  = 0;
volatile uint32_t can_error_code  = 0;
volatile uint32_t can_error_count = 0;
volatile uint32_t can_psr         = 0;
volatile uint32_t can_psr_tec     = 0;
volatile uint32_t can_busoff_cnt  = 0;
volatile uint8_t  can_rx_flag     = 0;

FDCAN_RxHeaderTypeDef can_rx_header;
uint8_t can_rx_data[8];

static MotorStatusCache_t s_motor_status[CAN_MOTOR_MAX_ID + 1U];

static uint32_t fdcan_dlc_from_len(uint8_t len)
{
    switch (len)
    {
        case 0: return FDCAN_DLC_BYTES_0;
        case 1: return FDCAN_DLC_BYTES_1;
        case 2: return FDCAN_DLC_BYTES_2;
        case 3: return FDCAN_DLC_BYTES_3;
        case 4: return FDCAN_DLC_BYTES_4;
        case 5: return FDCAN_DLC_BYTES_5;
        case 6: return FDCAN_DLC_BYTES_6;
        case 7: return FDCAN_DLC_BYTES_7;
        default: return FDCAN_DLC_BYTES_8;
    }
}

static uint8_t FDCAN_WaitFreeTxFifo(uint32_t timeout_ms)
{
    uint32_t start = HAL_GetTick();
    while (HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan2) == 0U)
    {
        if ((uint32_t)(HAL_GetTick() - start) > timeout_ms)
            return 0U;
    }
    return 1U;
}

uint8_t can_SendCmd(__IO uint8_t *cmd, uint8_t len)
{
    uint8_t i = 0U;
    uint8_t remaining;
    uint8_t l;
    uint8_t packNum = 0U;
    uint8_t data_bytes;
    FDCAN_TxHeaderTypeDef txHeader = {0};
    uint8_t txData[8] = {0};

    if (cmd == NULL || len < 3U)
        return 0U;

    /*
     * CAN1_MAP mapping used by the Emm firmware:
     * extended identifier = motor_address << 8 | packet_number
     * CAN data[0] repeats the command byte cmd[1].
     */
    remaining = (uint8_t)(len - 2U);

    while (i < remaining)
    {
        uint8_t left = (uint8_t)(remaining - i);
        data_bytes = (left > 7U) ? 7U : left;

        txHeader.Identifier = ((uint32_t)cmd[0] << 8) | (uint32_t)packNum;
        txHeader.IdType = FDCAN_EXTENDED_ID;
        txHeader.TxFrameType = FDCAN_DATA_FRAME;
        txHeader.DataLength = fdcan_dlc_from_len((uint8_t)(data_bytes + 1U));
        txHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
        txHeader.BitRateSwitch = FDCAN_BRS_OFF;
        txHeader.FDFormat = FDCAN_CLASSIC_CAN;
        txHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
        txHeader.MessageMarker = 0U;

        txData[0] = cmd[1];
        for (l = 0U; l < data_bytes; ++l)
            txData[l + 1U] = cmd[i + 2U + l];

        if (!FDCAN_WaitFreeTxFifo(FDCAN_TX_TIMEOUT_MS))
        {
            can_error_step = 1U;
            can_error_code = HAL_FDCAN_GetError(&hfdcan2);
            can_error_count++;
            return 0U;
        }

        if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &txHeader, txData) != HAL_OK)
        {
            can_error_step = 2U;
            can_error_code = HAL_FDCAN_GetError(&hfdcan2);
            can_error_count++;
            return 0U;
        }

        i = (uint8_t)(i + data_bytes);
        packNum++;
    }

    can_error_step = 0U;
    return 1U;
}

void can_motor_status_clear(uint8_t id)
{
    if (id == 0U || id > CAN_MOTOR_MAX_ID) return;
    s_motor_status[id].valid = 0U;
}

uint8_t can_motor_status_get(uint8_t id, uint8_t *status, uint32_t *tick_ms)
{
    uint32_t primask;
    uint8_t valid;

    if (id == 0U || id > CAN_MOTOR_MAX_ID || status == NULL)
        return 0U;

    primask = __get_PRIMASK();
    __disable_irq();
    valid = s_motor_status[id].valid;
    if (valid)
    {
        *status = s_motor_status[id].status;
        if (tick_ms) *tick_ms = s_motor_status[id].tick_ms;
    }
    if (!primask) __enable_irq();

    return valid;
}

uint8_t can_send_status_query(uint8_t id)
{
    uint8_t cmd[3] = { id, 0x3AU, 0x6BU };
    return can_SendCmd(cmd, sizeof(cmd));
}

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    if (hfdcan == NULL || hfdcan->Instance != FDCAN2)
        return;

    if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) == 0U)
        return;

    while (HAL_FDCAN_GetRxFifoFillLevel(hfdcan, FDCAN_RX_FIFO0) > 0U)
    {
        if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0,
                                   &can_rx_header, can_rx_data) == HAL_OK)
        {
            uint8_t id = 0U;
            can_rx_flag = 1U;

            if (can_rx_header.IdType == FDCAN_EXTENDED_ID)
                id = (uint8_t)(can_rx_header.Identifier >> 8);

            /*
             * Emm S_FLAG(0x3A) reply.
             *
             * UART protocol reply is:
             *   [Addr] [0x3A] [status] [0x6B]
             *
             * In this project's CAN1_MAP transport, Addr is carried in the
             * extended CAN identifier, so the CAN payload is:
             *   data[0] = 0x3A
             *   data[1] = status
             *   data[2] = 0x6B
             *
             * status bit1 (0x02) = position reached (Prf_TF).
             */
            if (id >= 1U && id <= CAN_MOTOR_MAX_ID &&
                can_rx_data[0] == 0x3AU &&
                can_rx_data[2] == 0x6BU)
            {
                s_motor_status[id].status = can_rx_data[1];
                s_motor_status[id].tick_ms = HAL_GetTick();
                s_motor_status[id].valid = 1U;
            }

        }
    }
}
  
uint8_t Emm_V5_Read_Status(uint8_t id, uint8_t *status, uint32_t timeout_ms)
{
    uint32_t start;
    uint32_t rx_tick;

    if (status == NULL || id == 0U || id > CAN_MOTOR_MAX_ID)
        return 0U;

    can_motor_status_clear(id);
    if (!can_send_status_query(id))
        return 0U;

    start = HAL_GetTick();
    while ((uint32_t)(HAL_GetTick() - start) < timeout_ms)
    {
        if (can_motor_status_get(id, status, &rx_tick))
            return 1U;
    }
    return 0U;
}

uint8_t Emm_V5_Is_Reached(uint8_t id)
{
    uint8_t status = 0U;
    if (!Emm_V5_Read_Status(id, &status, 50U))
        return 0U;
    return (status & 0x02U) ? 1U : 0U;
}

void fdcan2_UserInit(void)
{
    FDCAN_FilterTypeDef filter = {0};

    filter.IdType = FDCAN_EXTENDED_ID;
    filter.FilterIndex = 0;
    filter.FilterType = FDCAN_FILTER_MASK;
    filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    filter.FilterID1 = 0x00000000U;
    filter.FilterID2 = 0x00000000U;

    if (HAL_FDCAN_ConfigFilter(&hfdcan2, &filter) != HAL_OK)
        Error_Handler();

    if (HAL_FDCAN_ConfigGlobalFilter(&hfdcan2,
                                     FDCAN_REJECT,
                                     FDCAN_ACCEPT_IN_RX_FIFO0,
                                     FDCAN_REJECT_REMOTE,
                                     FDCAN_REJECT_REMOTE) != HAL_OK)
        Error_Handler();

    if (HAL_FDCAN_Start(&hfdcan2) != HAL_OK)
        Error_Handler();

    if (HAL_FDCAN_ActivateNotification(&hfdcan2,
            FDCAN_IT_RX_FIFO0_NEW_MESSAGE | FDCAN_IT_BUS_OFF, 0U) != HAL_OK)
        Error_Handler();
}

void HAL_FDCAN_ErrorCallback(FDCAN_HandleTypeDef *hfdcan)
{
    uint32_t psr;
    if (hfdcan == NULL || hfdcan->Instance != FDCAN2)
        return;

    psr = hfdcan->Instance->PSR;
    can_error_count++;
    can_psr = psr;
    can_psr_tec = (hfdcan->Instance->ECR & FDCAN_ECR_TEC);
    can_error_code = HAL_FDCAN_GetError(hfdcan);

    if ((psr & FDCAN_PSR_BO) != 0U)
        can_busoff_cnt++;
}

uint8_t fdcan2_recover(void)
{
    if (HAL_FDCAN_Stop(&hfdcan2) != HAL_OK)
        return 0U;
    if (HAL_FDCAN_Start(&hfdcan2) != HAL_OK)
        return 0U;
    if (HAL_FDCAN_ActivateNotification(&hfdcan2,
            FDCAN_IT_RX_FIFO0_NEW_MESSAGE | FDCAN_IT_BUS_OFF, 0U) != HAL_OK)
        return 0U;
    return 1U;
}
