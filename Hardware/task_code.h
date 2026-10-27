#ifndef __TASK_CODE_H
#define __TASK_CODE_H

#include <stdbool.h>
#include <stdint.h>

#define TASK_CODE_BATCH_COUNT  2U
#define TASK_CODE_ITEM_COUNT   3U
#define TASK_CODE_TEXT_LEN     15U

typedef struct
{
    /*
     * 颜色编号：
     *   1=红色，2=黄色，3=蓝色，4=绿色，5=黑色，6=浅蓝色。
     * 数组下标代表本批次的搬运次序，因此不能再按颜色号大小排序。
     */
    uint8_t color;

    /* 粗加工区目标圆环号，范围1~3，来自任务码第二组或第四组。 */
    uint8_t rough_slot;

    /*
     * 暂存区目标圆环号，范围1~3。
     * 第一轮直接来自任务码第二组；第二轮不使用第四组，而是按颜色查找
     * 第一轮同色物料，再继承该物料由第二组指定的圆环号，以完成同色叠放。
     */
    uint8_t storage_slot;
} MaterialTaskItem_t;

typedef struct
{
    /* batch[0]=第一批，batch[1]=第二批；每批下标0~2就是实际执行顺序。 */
    MaterialTaskItem_t batch[TASK_CODE_BATCH_COUNT][TASK_CODE_ITEM_COUNT];

    /* 保存校验通过后的原始15字节任务码，供HMI原样显示。 */
    char text[TASK_CODE_TEXT_LEN + 1U];

    /* 只有完整通过格式和映射校验后才置1，区域动作以此作为安全闸门。 */
    uint8_t valid;
} MissionTaskCode_t;

/**
 * @brief 解析完整比赛任务码。
 *
 * 固定格式为 CCC+PPP+CCC+PPP，共15个ASCII字符，例如：
 *   156+123+516+231
 *
 * 四组数据依次表示：
 *   1. 第一批三块物料的颜色及抓取顺序；
 *   2. 第一批在粗加工区、暂存区的放置圆环号；
 *   3. 第二批三块物料的颜色及抓取顺序；
 *   4. 第二批仅在粗加工区的放置圆环号。
 *
 * 校验规则：
 *   - 颜色只能为1~6，每一批内部不能重复；
 *   - 抽签确定整场比赛使用的三种颜色，因此两批颜色集合必须相同，
 *     但第三组可以改变这三种颜色在第二批中的搬运顺序；
 *   - 每个位置组必须是1、2、3的一种排列，不能漏位或重复占位；
 *   - 不接受前后多余字符，串口字节流的切帧由qr_scanner负责。
 *
 * @param text 以'\0'结束的任务码字符串。
 * @param out  解析成功后写入完整的两批颜色和位置映射。
 * @return true=格式及语义均合法；false=任务码无效，out会被清零。
 */
bool TaskCode_Parse(const char *text, MissionTaskCode_t *out);

/** @brief 安全读取某一批、某一次搬运对应的颜色及目标位置。 */
bool TaskCode_GetItem(const MissionTaskCode_t *task,
                      uint8_t batch,
                      uint8_t index,
                      MaterialTaskItem_t *out);

/**
 * @brief 查询第一批放在指定暂存圆环上的颜色。
 *
 * 第二批暂存区在固定2号圆做STACK视觉对位时，2号数字可能被第一层物料
 * 遮挡，因此需要用这里返回的实际颜色确认“哪个框是2号位”。
 */
bool TaskCode_GetStorageColor(const MissionTaskCode_t *task,
                              uint8_t storage_slot,
                              uint8_t *color);

/**
 * @brief 仅检查前15字节是否具有任务码线格式。
 *
 * 本函数只用于无包头包尾串口流的快速找帧，不检查组内重复；
 * 找到候选帧后仍必须调用TaskCode_Parse()完成严格校验。
 */
bool TaskCode_HasWireShape(const char *text);

#endif
