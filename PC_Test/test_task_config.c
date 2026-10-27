#include <math.h>
#include <stdio.h>

#include "manipulator_action.h"
#include "robot_config.h"

/*
 * 正式任务关键静态配置回归测试。
 *
 * 这里不模拟电机通信，只验证最容易被“占位值”悄悄破坏的关键配置：
 *   1. 第二批不再重复第一批的1/2/3颜色；
 *   2. 暂存区第二批位姿确实携带layer=1；
 *   3. 60 mm层高补偿会把下降量从-185 mm改成-125 mm，而不是仍落地；
 *   4. 粗加工区三个夹爪落点使用2026-10-22实测参数；
 *   5. 粗加工区和暂存区都保持“2号相机95 mm / 2号夹爪70 mm”分离。
 */

static int s_pass;
static int s_fail;

static void check(int condition, const char *name)
{
    printf("[%s] %s\n", condition ? "PASS" : "FAIL", name);
    if (condition) s_pass++;
    else s_fail++;
}

static int nearf_local(float a, float b)
{
    return fabsf(a - b) <= 0.001f;
}

int main(void)
{
    const ManipPose_t rough[3] =
    {
        ZONE_ROUGH_POS1_POSE,
        ZONE_ROUGH_POS2_POSE,
        ZONE_ROUGH_POS3_POSE
    };
    const ManipPose_t layer1[3] =
    {
        ZONE_STORAGE_POS1_POSE,
        ZONE_STORAGE_POS2_POSE,
        ZONE_STORAGE_POS3_POSE
    };
    const ManipPose_t layer2[3] =
    {
        ZONE_STORAGE_POS1_L2_POSE,
        ZONE_STORAGE_POS2_L2_POSE,
        ZONE_STORAGE_POS3_L2_POSE
    };
    unsigned int i;

    check(K230_RAW_ALIGN_B1_COLOR_1 == 1U &&
          K230_RAW_ALIGN_B1_COLOR_2 == 2U &&
          K230_RAW_ALIGN_B1_COLOR_3 == 3U,
          "Batch 1 keeps colors 1/2/3");
    check(K230_RAW_ALIGN_B2_COLOR_1 == 4U &&
          K230_RAW_ALIGN_B2_COLOR_2 == 5U &&
          K230_RAW_ALIGN_B2_COLOR_3 == 6U,
          "Batch 2 uses colors 4/5/6 without duplicates");
    check(nearf_local(MANIP_LAYER_STEP_MM, 60.0f),
          "Second-layer lift is the restored 60 mm value");
    check(nearf_local(MANIP_PICKUP_GROUND_MM + MANIP_LAYER_STEP_MM,
                      -125.0f),
          "Layer 2 resolves to a -125 mm gimbal target");

    check(nearf_local(ROUGH_VISION_LOW_SLIDE_POS2_MM, 95.0f),
          "Rough low-camera ring-2 alignment remains at 95 mm");
    check(nearf_local(rough[0].carousel_deg, 304.67f) &&
          nearf_local(rough[0].slide_extend_mm, 119.0f) &&
          nearf_local(rough[1].carousel_deg, 273.33f) &&
          nearf_local(rough[1].slide_extend_mm, 70.0f) &&
          nearf_local(rough[2].carousel_deg, 241.33f) &&
          nearf_local(rough[2].slide_extend_mm, 115.0f),
          "Rough formal poses use the measured gripper-center parameters");
    check(!nearf_local(rough[1].slide_extend_mm,
                       ROUGH_VISION_LOW_SLIDE_POS2_MM),
          "Rough ring-2 gripper pose stays separate from the camera pose");
    check(nearf_local(STORAGE_VISION_LOW_SLIDE_POS2_MM, 95.0f),
          "Storage low-camera ring-2 baseline starts at 95 mm");
    check(nearf_local(layer1[0].carousel_deg, 304.67f) &&
          nearf_local(layer1[0].slide_extend_mm, 119.0f) &&
          nearf_local(layer1[1].carousel_deg, 273.33f) &&
          nearf_local(layer1[1].slide_extend_mm, 70.0f) &&
          nearf_local(layer1[2].carousel_deg, 241.33f) &&
          nearf_local(layer1[2].slide_extend_mm, 115.0f),
          "Storage formal poses inherit the rough measured baseline");
    check(!nearf_local(layer1[1].slide_extend_mm,
                       STORAGE_VISION_LOW_SLIDE_POS2_MM),
          "Storage ring-2 gripper pose stays separate from the camera pose");

    for (i = 0U; i < 3U; i++)
    {
        check(layer1[i].layer == 0U && layer2[i].layer == 1U,
              "Storage layer tables distinguish batch 1 from batch 2");
        check(nearf_local(layer1[i].carousel_deg, layer2[i].carousel_deg) &&
              nearf_local(layer1[i].slide_extend_mm,
                          layer2[i].slide_extend_mm),
              "Both layers share the calibrated horizontal ring pose");
    }

    printf("PASS: %d\nFAIL: %d\n", s_pass, s_fail);
    return (s_fail == 0) ? 0 : 1;
}
