#ifndef LED_CONTROL_H
#define LED_CONTROL_H

#include <stdint.h>
#include "pogobase.h"


// Fonctions pour contrôler les moteurs
void led_set_color(uint8_t r, uint8_t g, uint8_t b);
void led_set_red(void);
void led_set_green(void);
void led_set_blue(void);
void led_set_off(void);
void led_blink(uint8_t r, uint8_t g, uint8_t b, int duration);

#endif // LED_CONTROL_H