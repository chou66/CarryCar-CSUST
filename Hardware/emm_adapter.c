#include "emm_adapter.h"
#include "can.h"

static HAL_StatusTypeDef send_raw(uint8_t *cmd, uint8_t len)
{
    return can_SendCmd(cmd, len) ? HAL_OK : HAL_BUSY;
}

HAL_StatusTypeDef emm_vel(uint8_t id, uint8_t dir, uint16_t rpm,
                          uint8_t acc, bool sync)
{
    uint8_t cmd[8] = {
        id, 0xF6U, dir,
        (uint8_t)(rpm >> 8), (uint8_t)rpm,
        acc, (uint8_t)sync, 0x6BU
    };
    return send_raw(cmd, sizeof(cmd));
}

HAL_StatusTypeDef emm_pos(uint8_t id, uint8_t dir, uint16_t speed,
                          uint8_t acc, uint32_t clk, bool raF, bool snF)
{
    uint8_t cmd[13] = {
        id, 0xFDU, dir,
        (uint8_t)(speed >> 8), (uint8_t)speed,
        acc,
        (uint8_t)(clk >> 24), (uint8_t)(clk >> 16),
        (uint8_t)(clk >> 8), (uint8_t)clk,
        (uint8_t)raF, (uint8_t)snF, 0x6BU
    };
    return send_raw(cmd, sizeof(cmd));
}

HAL_StatusTypeDef emm_stop(uint8_t id, bool sync)
{
    uint8_t cmd[5] = { id, 0xFEU, 0x98U, (uint8_t)sync, 0x6BU };
    return send_raw(cmd, sizeof(cmd));
}

HAL_StatusTypeDef emm_read_status(uint8_t id)
{
    return can_send_status_query(id) ? HAL_OK : HAL_BUSY;
}

HAL_StatusTypeDef emm_sync_start(void)
{
    uint8_t cmd[4] = { 0U, 0xFFU, 0x66U, 0x6BU };
    return send_raw(cmd, sizeof(cmd));
}

void emm_prepare_position_feedback(uint8_t id)
{
    can_motor_status_clear(id);
}

bool emm_all_position_reached_after(uint8_t mask, uint32_t start_ms)
{
    uint8_t id;

    for (id = 1U; id <= CAN_MOTOR_MAX_ID; ++id)
    {
        uint8_t bit = (uint8_t)(1U << (id - 1U));
        uint8_t status;
        uint32_t tick;

        if ((mask & bit) == 0U)
            continue;

        if (!can_motor_status_get(id, &status, &tick))
            return false;

        if ((int32_t)(tick - start_ms) < 0)
            return false;

        if ((status & 0x02U) == 0U)
            return false;
    }
    return true;
}

bool emm_any_position_fault_after(uint8_t mask, uint32_t start_ms)
{
    uint8_t id;

    /*
     * S_FLAG:
     *   bit1 0x02 = Prf_TF, position reached
     *   bit2 0x04 = Cgi_TF, stall detected
     *   bit3 0x08 = Cgp_TF, stall protection triggered
     */
    for (id = 1U; id <= CAN_MOTOR_MAX_ID; ++id)
    {
        uint8_t bit = (uint8_t)(1U << (id - 1U));
        uint8_t status;
        uint32_t tick;

        if ((mask & bit) == 0U)
            continue;

        if (!can_motor_status_get(id, &status, &tick))
            continue;

        if ((int32_t)(tick - start_ms) < 0)
            continue;

        if ((status & 0x0CU) != 0U)
            return true;
    }

    return false;
}

bool emm_any_stall_protect_after(uint8_t mask, uint32_t start_ms)
{
    uint8_t id;

    /*
     * 与底盘 emm_any_position_fault_after 的差别仅在判据：
     * 只认 0x08(Cgp_TF 堵转保护锁存)。0x04(Cgi_TF 堵转检测)是夹爪
     * 落座压料瞬间的正常警告，不得抢占到位确认。
     */
    for (id = 1U; id <= CAN_MOTOR_MAX_ID; ++id)
    {
        uint8_t bit = (uint8_t)(1U << (id - 1U));
        uint8_t status;
        uint32_t tick;

        if ((mask & bit) == 0U)
            continue;

        if (!can_motor_status_get(id, &status, &tick))
            continue;

        if ((int32_t)(tick - start_ms) < 0)
            continue;

        if ((status & 0x08U) != 0U)
            return true;
    }

    return false;
}

uint8_t emm_reached_mask_after(uint8_t mask, uint32_t start_ms)
{
    uint8_t id;
    uint8_t result = 0U;

    for (id = 1U; id <= CAN_MOTOR_MAX_ID; ++id)
    {
        uint8_t bit = (uint8_t)(1U << (id - 1U));
        uint8_t status;
        uint32_t tick;

        if ((mask & bit) == 0U)
            continue;

        if (can_motor_status_get(id, &status, &tick) &&
            ((int32_t)(tick - start_ms) >= 0) &&
            ((status & 0x02U) != 0U))
        {
            result |= bit;
        }
    }

    return result;
}

uint8_t emm_faulted_mask_after(uint8_t mask, uint32_t start_ms)
{
    uint8_t id;
    uint8_t result = 0U;

    for (id = 1U; id <= CAN_MOTOR_MAX_ID; ++id)
    {
        uint8_t bit = (uint8_t)(1U << (id - 1U));
        uint8_t status;
        uint32_t tick;

        if ((mask & bit) == 0U)
            continue;

        if (can_motor_status_get(id, &status, &tick) &&
            ((int32_t)(tick - start_ms) >= 0) &&
            ((status & 0x08U) != 0U))
        {
            result |= bit;
        }
    }

    return result;
}

