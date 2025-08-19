#include "board.h"
#include "main.h"
#include "stm32f4xx_hal.h"
#include "lin.h"
#include "usbd_cdc_if.h"
#include "ina260.h"
#include <stdbool.h>   // <-- add this
#include <stdio.h>     // <-- for snprintf
// ---- decode + lock state ----
typedef enum {
  ACT_NONE = 0,
  ACT_A_THIGH_EXT, ACT_A_THIGH_RET,
  ACT_B_SEAT_FOR,  ACT_B_SEAT_REV,
  ACT_B_SEAT_UP,   ACT_B_SEAT_DN,
  ACT_B_TILT_UP,   ACT_B_TILT_DN,
  ACT_C_BACKREST_TILT_UP, ACT_C_BACKREST_TILT_DN,
} SeatAction_t;

static SeatAction_t s_locked = ACT_NONE;

static const char* action_name(SeatAction_t a) {
  switch (a) {
    case ACT_A_THIGH_EXT: 			return "A_THIGH_EXT";
    case ACT_A_THIGH_RET: 			return "A_THIGH_RET";
    case ACT_B_SEAT_FOR:  			return "B_SEAT_FOR";
    case ACT_B_SEAT_REV:  			return "B_SEAT_REV";
    case ACT_B_SEAT_UP:   			return "B_SEAT_UP";
    case ACT_B_SEAT_DN:   			return "B_SEAT_DN";
    case ACT_B_TILT_UP:   			return "B_SEAT_TILT_UP";
    case ACT_B_TILT_DN:   			return "B_SEAT_TILT_DN";
    case ACT_C_BACKREST_TILT_UP: 	return "C_BACKREST_TILT_UP";
    case ACT_C_BACKREST_TILT_DN: 	return "C_BACKREST_TILT_DN";
    default: return "NONE";
  }
}

static SeatAction_t decode_action(uint8_t d0, uint8_t d1, uint8_t d2, bool *is_nominal)
{
  *is_nominal = (d0 == 0x00 && d1 == 0x0C && d2 == 0xC0);

  // A_* (unique by DATA1 = 0x1C/0x2C, DATA2 = 0xC0)
  if (d2 == 0xC0) {
    if (d1 == 0x1C) return ACT_A_THIGH_EXT;   // 00 1C C0
    if (d1 == 0x2C) return ACT_A_THIGH_RET;   // 00 2C C0
  }

  // B_* group and C group (DATA1 = 0x0C, DATA2 = 0xC0).
  //Choose first set bit (priority order).
  if (d1 == 0x0C && d2 == 0xC0) {
    if (d0 & 0x01) return ACT_B_SEAT_FOR;   // 01 0C C0
    if (d0 & 0x02) return ACT_B_SEAT_REV;   // 02 0C C0
    if (d0 & 0x04) return ACT_B_SEAT_UP;    // 04 0C C0
    if (d0 & 0x08) return ACT_B_SEAT_DN;    // 08 0C C0
    if (d0 & 0x40) return ACT_B_TILT_UP;    // 40 0C C0
    if (d0 & 0x80) return ACT_B_TILT_DN;    // 80 0C C0
	if (d0 & 0x20) return ACT_C_BACKREST_TILT_UP;  //20 0C C0
	if (d0 & 0x10) return ACT_C_BACKREST_TILT_DN; //10 0C C0
  }


  return ACT_NONE;
}

static void drive_for_action(SeatAction_t a)
{
  // Ensure only one motor is driven
  SetHalfBridge(IDLE_MOTOR, DIR_IDLE);

  HalfBridge_t hb = HB1;
  Direction_t  dir = DIR_IDLE;

  switch (a) {
    case ACT_A_THIGH_EXT: hb = HB5; dir = DIR_FORWARD; break;
    case ACT_A_THIGH_RET: hb = HB5; dir = DIR_REVERSE; break;

    case ACT_B_SEAT_FOR:  hb = HB2; dir = DIR_FORWARD; break;
    case ACT_B_SEAT_REV:  hb = HB2; dir = DIR_REVERSE; break;

    case ACT_B_SEAT_UP:   hb = HB1; dir = DIR_FORWARD; break;
    case ACT_B_SEAT_DN:   hb = HB1; dir = DIR_REVERSE; break;

    case ACT_B_TILT_UP:   hb = HB3; dir = DIR_FORWARD; break;
    case ACT_B_TILT_DN:   hb = HB3; dir = DIR_REVERSE; break;

    case ACT_C_BACKREST_TILT_UP: hb = HB4; dir = DIR_FORWARD; break;
    case ACT_C_BACKREST_TILT_DN: hb = HB4; dir = DIR_REVERSE; break;

    default: break;
  }

  if (dir != DIR_IDLE) {
    SetHalfBridge(hb, dir);
  }
}

void Board_ReadLinAndDrive(void)
{
  // Poll ID 0x01 and parse DATA0..2
  uint8_t rx[9];
  if (lin_send_header(&huart2, 0x01) != HAL_OK) {
    CDC_Transmit_FS((uint8_t*)"LIN hdr fail\r\n", 13);
    return;
  }
  int n = lin_receive_response(&huart2, rx, sizeof(rx), 80);
  if (n < 3) return;

  uint8_t d0 = rx[0], d1 = rx[1], d2 = rx[2];
  bool nominal = false;
  SeatAction_t act = decode_action(d0, d1, d2, &nominal);

  // Release lock on nominal
  if (nominal) {
    if (s_locked != ACT_NONE) {
      SetHalfBridge(IDLE_MOTOR, DIR_IDLE);
      s_locked = ACT_NONE;
      CDC_Transmit_FS((uint8_t*)"FUNC: IDLE\r\n", 12);
    }
    return;
  }

  // Already locked? ignore any other inputs
  if (s_locked != ACT_NONE) return;

  // Acquire lock on first recognized function
  if (act != ACT_NONE) {
    s_locked = act;
    drive_for_action(act);

    char msg[48];
    int len = snprintf(msg, sizeof(msg), "FUNC: %s\r\n", action_name(act));
    CDC_Transmit_FS((uint8_t*)msg, (uint16_t)len);
  }
}



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
