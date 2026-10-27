#include <stdio.h>
#include <string.h>

#include "qr_scanner.h"
#include "task_code.h"

static int s_fail;

static void check(int condition, const char *name)
{
    printf("[%s] %s\n", condition ? "PASS" : "FAIL", name);
    if (!condition) s_fail++;
}

static void feed(const char *text, unsigned int *now_ms)
{
    while (*text)
    {
        QRScanner_FeedByte((unsigned char)*text++, (*now_ms)++);
    }
}

int main(void)
{
    MissionTaskCode_t task;
    MaterialTaskItem_t item;
    char code[QR_CODE_MAX + 1U];
    unsigned int now_ms = 1U;

    check(TaskCode_Parse("156+123+516+231", &task),
          "parse official example");
    check(task.valid && strcmp(task.text, "156+123+516+231") == 0,
          "preserve canonical task text");
    check(TaskCode_GetItem(&task, 0U, 1U, &item) &&
          item.color == 5U && item.rough_slot == 2U &&
          item.storage_slot == 2U,
          "batch 1 item maps color/order to physical slot");
    check(TaskCode_GetItem(&task, 1U, 0U, &item) &&
          item.color == 5U && item.rough_slot == 2U &&
          item.storage_slot == 2U,
          "batch 2 rough slot uses group 4 and storage follows same color");
    check(TaskCode_GetItem(&task, 1U, 1U, &item) &&
          item.color == 1U && item.rough_slot == 3U &&
          item.storage_slot == 1U,
          "batch 2 storage ignores group 4 and finds first-layer color");
    check(TaskCode_GetItem(&task, 1U, 2U, &item) &&
          item.color == 6U && item.rough_slot == 1U &&
          item.storage_slot == 3U,
          "batch 2 keeps independent rough and storage mappings");

    /* 竞赛命题解析P15给出的二维码示例，逐项验证四组数字的正式含义。 */
    check(TaskCode_Parse("452+321+254+312", &task),
          "parse task code shown in competition explanation");
    check(TaskCode_GetItem(&task, 1U, 0U, &item) &&
          item.color == 2U && item.rough_slot == 3U &&
          item.storage_slot == 1U,
          "slide example batch-2 yellow stacks on batch-1 yellow");
    check(TaskCode_GetItem(&task, 1U, 2U, &item) &&
          item.color == 4U && item.rough_slot == 2U &&
          item.storage_slot == 3U,
          "slide example batch-2 green ignores group 4 in storage zone");
    check(!TaskCode_Parse("156+123+516+211", &task),
          "reject non-permutation position group");
    check(!TaskCode_Parse("123+321+456+123", &task),
          "reject different color sets because same-color stacking is impossible");
    check(TaskCode_Parse("123+321+312+123", &task),
          "accept the same three colors in a different batch-2 order");
    check(TaskCode_GetItem(&task, 1U, 1U, &item) &&
          item.color == 1U && item.rough_slot == 2U &&
          item.storage_slot == 3U,
          "reordered batch 2 maps storage by color instead of item index");
    check(!TaskCode_Parse("156-123+516+231", &task),
          "reject malformed separators");
    check(!TaskCode_Parse("156", &task),
          "reject short input safely");
    check(!TaskCode_Parse("156+123+516+231X", &task),
          "reject trailing bytes at semantic parser");

    QRScanner_Init();
    feed("156+123+516+231", &now_ms);
    check(QRScanner_GetNewCode(code, sizeof(code)) &&
          strcmp(code, "156+123+516+231") == 0,
          "headerless frame completes at fixed length");

    feed("156+123+516+231156+123+516+231", &now_ms);
    check(!QRScanner_GetNewCode(code, sizeof(code)),
          "continuous duplicate scans are de-duplicated");

    feed("noise!123+321+312+132", &now_ms);
    check(QRScanner_GetNewCode(code, sizeof(code)) &&
          strcmp(code, "123+321+312+132") == 0,
          "stream window re-synchronizes without header/tail");

    feed("\r\n", &now_ms);
    check(!QRScanner_GetNewCode(code, sizeof(code)),
          "line endings do not create duplicate events");

    check(TaskCode_Parse(code, &task),
          "scanner output passes semantic parser");

    printf("FAIL: %d\n", s_fail);
    return s_fail ? 1 : 0;
}
