#include "motor_control.h"

void pogobot_move_forward(void) {
    pogobot_motor_set(motorL, motorFull);
    pogobot_motor_set(motorR, motorFull);
}

void pogobot_turn_left(void) {
    pogobot_motor_set(motorL, motorQuarter);
    pogobot_motor_set(motorR, motorThreeQuarter);
}

void pogobot_turn_right(void) {
    pogobot_motor_set(motorL, motorThreeQuarter);
    pogobot_motor_set(motorR, motorQuarter);
}

void pogobot_stop(void) {
    pogobot_motor_set(motorL, motorStop);
    pogobot_motor_set(motorR, motorStop);
}