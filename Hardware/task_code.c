#include "task_code.h"

#include <string.h>

static bool group_unique(const uint8_t value[TASK_CODE_ITEM_COUNT])
{
    /* 每组固定只有三个数，直接两两比较比引入额外查找表更小、更直观。 */
    return (value[0] != value[1]) &&
           (value[0] != value[2]) &&
           (value[1] != value[2]);
}

static bool same_color_set(const uint8_t first[TASK_CODE_ITEM_COUNT],
                           const uint8_t second[TASK_CODE_ITEM_COUNT])
{
    uint8_t first_mask = 0U;
    uint8_t second_mask = 0U;
    uint8_t i;

    /*
     * 颜色号范围已经由TaskCode_HasWireShape()限制为1~6，因此可以把每个
     * 颜色映射到一个bit。这里只比较“包含哪些颜色”，不比较出现顺序：
     * 例如第一批156、第二批516是合法的，因为两批都是红/黑/浅蓝，
     * 只是第二批的搬运顺序发生了变化。
     */
    for (i = 0U; i < TASK_CODE_ITEM_COUNT; ++i)
    {
        first_mask |= (uint8_t)(1U << (first[i] - 1U));
        second_mask |= (uint8_t)(1U << (second[i] - 1U));
    }

    return first_mask == second_mask;
}

bool TaskCode_HasWireShape(const char *text)
{
    uint8_t i;

    if (!text) return false;

    /* 先确认至少有15个字符，避免普通短字符串访问越界。 */
    for (i = 0U; i < TASK_CODE_TEXT_LEN; ++i)
    {
        if (text[i] == '\0') return false;
    }

    if (text[3] != '+' || text[7] != '+' || text[11] != '+') return false;

    for (i = 0U; i < TASK_CODE_TEXT_LEN; ++i)
    {
        if (i == 3U || i == 7U || i == 11U) continue;

        if (i < 3U || (i >= 8U && i < 11U))
        {
            if (text[i] < '1' || text[i] > '6') return false;
        }
        else
        {
            if (text[i] < '1' || text[i] > '3') return false;
        }
    }

    return true;
}

bool TaskCode_Parse(const char *text, MissionTaskCode_t *out)
{
    MissionTaskCode_t parsed;
    uint8_t colors[TASK_CODE_BATCH_COUNT][TASK_CODE_ITEM_COUNT];
    uint8_t slots[TASK_CODE_BATCH_COUNT][TASK_CODE_ITEM_COUNT];
    uint8_t batch;
    uint8_t i;

    if (!text || !out) return false;
    memset(out, 0, sizeof(*out));
    memset(&parsed, 0, sizeof(parsed));

    if (!TaskCode_HasWireShape(text) || text[TASK_CODE_TEXT_LEN] != '\0')
        return false;

    for (i = 0U; i < TASK_CODE_ITEM_COUNT; ++i)
    {
        colors[0][i] = (uint8_t)(text[i] - '0');
        slots[0][i] = (uint8_t)(text[4U + i] - '0');
        colors[1][i] = (uint8_t)(text[8U + i] - '0');
        slots[1][i] = (uint8_t)(text[12U + i] - '0');
    }

    if (!group_unique(colors[0]) ||
        !group_unique(colors[1]) ||
        !group_unique(slots[0]) ||
        !group_unique(slots[1]) ||
        !same_color_set(colors[0], colors[1]))
    {
        return false;
    }

    /*
     * 四组任务码不能简单地按“每批一组颜色+一组通用位置”处理：
     *   1. 第一组：第一批颜色及搬运顺序；
     *   2. 第二组：第一批在粗加工区、暂存区的放置位置；
     *   3. 第三组：第二批颜色及搬运顺序；
     *   4. 第四组：只表示第二批在粗加工区的放置位置。
     *
     * 第二批到暂存区后必须叠放在第一批的同颜色物料上，所以任务码没有
     * 单独给出第二批暂存位置。下面先装入两批的颜色和粗加工区位置；随后
     * 再按颜色回查第一批，把第一批同色物料的暂存圆环号赋给第二批。
     */
    for (batch = 0U; batch < TASK_CODE_BATCH_COUNT; ++batch)
    {
        for (i = 0U; i < TASK_CODE_ITEM_COUNT; ++i)
        {
            parsed.batch[batch][i].color = colors[batch][i];
            parsed.batch[batch][i].rough_slot = slots[batch][i];
        }
    }


    for (i = 0U; i < TASK_CODE_ITEM_COUNT; ++i)
    {
        uint8_t first_index;

        /* 第一批在暂存区与粗加工区共用第二组给出的物理圆环号。 */
        parsed.batch[0][i].storage_slot = slots[0][i];

        /*
         * 第二批第i块物料的暂存位置不取第四组，而由颜色决定：在第一批
         * 中找到相同颜色，再继承该第一批物料由第二组指定的暂存圆环号。
         * same_color_set()已保证一定能找到；保留循环写法便于调试时观察
         * “第二批颜色 -> 第一批下标 -> 暂存圆环号”的完整映射过程。
         */
        for (first_index = 0U;
             first_index < TASK_CODE_ITEM_COUNT;
             ++first_index)
        {
            if (colors[0][first_index] == colors[1][i])
            {
                parsed.batch[1][i].storage_slot = slots[0][first_index];
                break;
            }
        }
    }

    memcpy(parsed.text, text, TASK_CODE_TEXT_LEN + 1U);
    parsed.valid = 1U;
    *out = parsed;
    return true;
}

bool TaskCode_GetItem(const MissionTaskCode_t *task,
                      uint8_t batch,
                      uint8_t index,
                      MaterialTaskItem_t *out)
{
    /* 所有对外读取都检查valid和数组边界，防止非法任务进入机械状态机。 */
    if (!task || !out || !task->valid ||
        batch >= TASK_CODE_BATCH_COUNT || index >= TASK_CODE_ITEM_COUNT)
    {
        return false;
    }

    *out = task->batch[batch][index];
    return true;
}

bool TaskCode_GetStorageColor(const MissionTaskCode_t *task,
                              uint8_t storage_slot,
                              uint8_t *color)
{
    uint8_t i;

    if (!task || !color || !task->valid ||
        storage_slot < 1U || storage_slot > 3U)
    {
        return false;
    }

    /* storage_slot保存的是物理圆环号，因此按值查找，不能直接拿数组下标。 */
    for (i = 0U; i < TASK_CODE_ITEM_COUNT; ++i)
    {
        if (task->batch[0][i].storage_slot == storage_slot)
        {
            *color = task->batch[0][i].color;
            return true;
        }
    }
    return false;
}
