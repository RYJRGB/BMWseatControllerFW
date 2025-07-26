#ifndef BOARD_H
#define BOARD_H

typedef enum {
    IDLE_MOTOR = 0,
    HB1,
    HB2,
    HB3,
    HB4,
    HB5
} HalfBridge_t;

typedef enum {
    DIR_IDLE = 0,
    DIR_FORWARD,
    DIR_REVERSE
} Direction_t;

void SetHalfBridge(HalfBridge_t hb, Direction_t dir);

#endif // BOARD_H
