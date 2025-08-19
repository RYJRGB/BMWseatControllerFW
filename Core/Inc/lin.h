#ifndef LIN_H
#define LIN_H

#include "main.h"     // brings in HAL + UART_HandleTypeDef
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Provided by CubeMX elsewhere */
extern UART_HandleTypeDef huart2;

/* Public functions (moved out of main.c) */
void LIN_Ping_And_Print(void);

uint8_t make_lin_id_with_parity(uint8_t id6);
HAL_StatusTypeDef lin_send_header(UART_HandleTypeDef *huart, uint8_t id6);
int  lin_receive_response(UART_HandleTypeDef *huart, uint8_t *buf, int max_len, uint32_t timeout_ms);
void cdc_print_hex(const char *prefix, const uint8_t *data, int len);
void uart_flush_errors(UART_HandleTypeDef* hu);

#ifdef __cplusplus
}
#endif

#endif /* LIN_H */
