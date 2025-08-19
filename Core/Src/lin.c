#include "lin.h"
#include "usbd_cdc_if.h"
#include <stdio.h>
#include <string.h>

/* SYNC + PID query + print (ID = 0x01 → PID = 0xC1) */
void LIN_Ping_And_Print(void)
{
    uint8_t tx[2] = { 0x55, 0xC1 };   /* SYNC + PID for ID 0x01 */

    HAL_LIN_SendBreak(&huart2);
    HAL_UART_Transmit(&huart2, tx, 2, 20);

    /* Clear BREAK artifacts so first slave byte isn't lost */
    __HAL_UART_CLEAR_FEFLAG(&huart2);
    __HAL_UART_CLEAR_NEFLAG(&huart2);
    __HAL_UART_CLEAR_OREFLAG(&huart2);
#if defined(USART_RDR_RDR) || defined(USART_ISR_RXNE)  /* L4/G4 style */
    (void)huart2.Instance->ISR; (void)huart2.Instance->RDR;
#else                                                   /* F1/F2/F3/F4 style */
    (void)huart2.Instance->SR;  (void)huart2.Instance->DR;
#endif
    huart2.ErrorCode = HAL_UART_ERROR_NONE;

    uint8_t rx[9];                                      /* up to 8 data + 1 checksum */
    int n = lin_receive_response(&huart2, rx, sizeof(rx), 80);
    if (n <= 0) {
        CDC_Transmit_FS((uint8_t*)"LIN: no response\r\n", 18);
    } else {
        cdc_print_hex("LIN ID 0x01 -> ", rx, n);
    }
}

/* Parity per LIN 2.x: ID = b5..b0, P0 on b6, P1 on b7 */
uint8_t make_lin_id_with_parity(uint8_t id6)
{
    id6 &= 0x3F;
    uint8_t b0 = (id6 >> 0) & 1;
    uint8_t b1 = (id6 >> 1) & 1;
    uint8_t b2 = (id6 >> 2) & 1;
    uint8_t b3 = (id6 >> 3) & 1;
    uint8_t b4 = (id6 >> 4) & 1;
    uint8_t b5 = (id6 >> 5) & 1;

    uint8_t p0 = (b0 ^ b1 ^ b2 ^ b4) & 1;
    uint8_t p1 = (~(b1 ^ b3 ^ b4 ^ b5)) & 1;

    return (uint8_t)(id6 | (p0 << 6) | (p1 << 7));
}

/* Send BREAK + SYNC(0x55) + ID(with parity) — minimal version (no RX toggling) */
HAL_StatusTypeDef lin_send_header(UART_HandleTypeDef *huart, uint8_t id6)
{
    if (HAL_LIN_SendBreak(huart) != HAL_OK) return HAL_ERROR;

    uint8_t sync = 0x55;
    if (HAL_UART_Transmit(huart, &sync, 1, 5) != HAL_OK) return HAL_ERROR;

    uint8_t pid = make_lin_id_with_parity(id6);
    if (HAL_UART_Transmit(huart, &pid, 1, 5) != HAL_OK) return HAL_ERROR;

    /* Optional hygiene: clear any header-induced flags */
    __HAL_UART_CLEAR_FEFLAG(huart);
    __HAL_UART_CLEAR_NEFLAG(huart);
    __HAL_UART_CLEAR_OREFLAG(huart);
#if defined(USART_RDR_RDR) || defined(USART_ISR_RXNE)
    (void)huart->Instance->ISR; (void)huart->Instance->RDR;
#else
    (void)huart->Instance->SR;  (void)huart->Instance->DR;
#endif
    huart->ErrorCode = HAL_UART_ERROR_NONE;

    return HAL_OK;
}

/* Simple blocking read until idle timeout */
int lin_receive_response(UART_HandleTypeDef *huart, uint8_t *buf, int max_len, uint32_t timeout_ms)
{
    int n = 0;
    uint32_t t0 = HAL_GetTick();
    while (n < max_len) {
        if (HAL_UART_Receive(huart, &buf[n], 1, 1) == HAL_OK) {
            n++;
            t0 = HAL_GetTick();                           /* extend window on activity */
        }
        if ((HAL_GetTick() - t0) > timeout_ms) break;
    }
    return n;                                             /* 0 means no data */
}

void cdc_print_hex(const char *prefix, const uint8_t *data, int len)
{
    char line[128];
    int off = snprintf(line, sizeof(line), "%s", prefix);
    for (int i = 0; i < len && off < (int)sizeof(line) - 4; ++i) {
        off += snprintf(line + off, sizeof(line) - off, "%02X ", data[i]);
    }
    if (off < (int)sizeof(line) - 2) { line[off++] = '\r'; line[off++] = '\n'; }
    CDC_Transmit_FS((uint8_t*)line, off);
}

void uart_flush_errors(UART_HandleTypeDef* hu)
{
  __HAL_UART_CLEAR_FEFLAG(hu);
  __HAL_UART_CLEAR_NEFLAG(hu);
  __HAL_UART_CLEAR_OREFLAG(hu);
#if defined(USART_RDR_RDR) || defined(USART_ISR_RXNE)  /* L4/G4 style */
  (void)hu->Instance->ISR; (void)hu->Instance->RDR;
#else                                                   /* F1/F2/F3/F4 style */
  (void)hu->Instance->SR;  (void)hu->Instance->DR;
#endif
  hu->ErrorCode = HAL_UART_ERROR_NONE;
}
