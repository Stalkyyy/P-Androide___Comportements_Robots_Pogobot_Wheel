#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include <stdint.h>
#include "pogobase.h"


// Fonctions pour contrôler les moteurs
void pogobot_move_forward(void);
void pogobot_move_backward(void);
void pogobot_turn_left(void);
void pogobot_turn_right(void);
void pogobot_stop(void);

#endif // MOTOR_CONTROL_H