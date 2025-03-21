#include "led_control.h"

void led_set_color(uint8_t r, uint8_t g, uint8_t b) {
    pogobot_led_setColor(r, g, b);
}

void led_set_red(void) {
    led_set_color(255, 0, 0);
}

void led_set_green(void) {
    led_set_color(0, 255, 0);
}

void led_set_blue(void) {
    led_set_color(0, 0, 255);
}

void led_set_off(void) {
    led_set_color(0, 0, 0);
}

void led_blink(uint8_t r, uint8_t g, uint8_t b, int duration) {
    led_set_color(r, g, b);
    msleep(duration);
    led_set_off();
    msleep(duration);
}