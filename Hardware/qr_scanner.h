#ifndef __QR_SCANNER_H
#define __QR_SCANNER_H

#include <stdint.h>

#define QR_CODE_MAX 63U

/*
 * USART2中断持续逐字节喂入本模块。扫码器不需要提供二进制包头、包尾，
 * 模块会按比赛任务码的固定15字节结构自动找帧，并对连续重复结果去重。
 */
void QRScanner_Init(void);
void QRScanner_FeedByte(uint8_t byte, uint32_t now_ms);
void QRScanner_Update(uint32_t now_ms);
uint8_t QRScanner_GetNewCode(char *out, uint16_t max_len);
uint8_t QRScanner_GetLatestCode(char *out, uint16_t max_len);
uint8_t QRScanner_IsOnline(uint32_t now_ms);

#endif
