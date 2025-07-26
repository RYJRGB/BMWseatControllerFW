#include "board.h"
#include "main.h"
#include "stm32f4xx_hal.h"

void SetHalfBridge(HalfBridge_t hb, Direction_t dir)
{
    GPIO_TypeDef *portA[] = {GPIOA, GPIOA, GPIOA, GPIOB, GPIOB};
    uint16_t pinA[] = {BR1A_Pin, BR2A_Pin, BR3A_Pin, BR4A_Pin, BR5A_Pin};
    GPIO_TypeDef *portB[] = {GPIOA, GPIOA, GPIOA, GPIOB, GPIOB};
    uint16_t pinB[] = {BR1B_Pin, BR2B_Pin, BR3B_Pin, BR4B_Pin, BR5B_Pin};

    if (hb == IDLE_MOTOR) {
        // Set all half-bridges to idle
        for (int i = 0; i < 5; i++) {
            HAL_GPIO_WritePin(portA[i], pinA[i], GPIO_PIN_RESET);
            HAL_GPIO_WritePin(portB[i], pinB[i], GPIO_PIN_RESET);
        }
        return;
    }

    if (hb < HB1 || hb > HB5) return; // Invalid input guard

    int index = hb - HB1;

    switch (dir) {
        case DIR_IDLE:
            HAL_GPIO_WritePin(portA[index], pinA[index], GPIO_PIN_RESET);
            HAL_GPIO_WritePin(portB[index], pinB[index], GPIO_PIN_RESET);
            break;
        case DIR_FORWARD:
            HAL_GPIO_WritePin(portA[index], pinA[index], GPIO_PIN_SET);
            HAL_GPIO_WritePin(portB[index], pinB[index], GPIO_PIN_RESET);
            break;
        case DIR_REVERSE:
            HAL_GPIO_WritePin(portA[index], pinA[index], GPIO_PIN_RESET);
            HAL_GPIO_WritePin(portB[index], pinB[index], GPIO_PIN_SET);
            break;
        default:
            break;
    }
}
