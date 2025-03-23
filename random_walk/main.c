#include "pogobase.h"
#include <stdlib.h>
#include <time.h>

#define MIN_DISTANCE 0
#define ACTION_DELAY_MICROSECONDS 16

typedef struct {
    uint8_t data_foo[8];
    time_reference_t timer_it;
} USERDATA;

DECLARE_USERDATA(USERDATA);
REGISTER_USERDATA(USERDATA);

void user_init(void) {
#ifndef SIMULATOR
    printf("setup ok\n");
#endif
    pogobot_stopwatch_reset(&mydata->timer_it);
    main_loop_hz = 60;
    max_nb_processed_msg_per_tick = 0;
    msg_rx_fn = NULL;
    msg_tx_fn = NULL;
    error_codes_led_idx = 3;
    pogobot_infrared_set_power(3);
    srand(pogobot_helper_getRandSeed()); // Initialiser le générateur de nombres aléatoires
}

void ping_robots(void) {
    uint8_t ping_message[1] = {0};
    pogobot_infrared_sendLongMessage_omniSpe(ping_message, sizeof(ping_message));
}

void get_intensities(int intensities[]) {
    pogobot_infrared_update();
    while (pogobot_infrared_message_available()) {
        message_t msg;
        pogobot_infrared_recover_next_message(&msg);
        int sensor_id = msg.header._receiver_ir_index;
        int power = msg.header._emitting_power_list;
        printf("%d\n", sensor_id);
        if (sensor_id >= 0 && sensor_id < 4) {
            intensities[sensor_id] = power;
        }
    }
}

bool avoid_collision(time_reference_t *timer, int *intensities) {
    if (intensities[0] > MIN_DISTANCE || intensities[1] > MIN_DISTANCE || intensities[3] > MIN_DISTANCE) {
        pogobot_timer_init(timer, ACTION_DELAY_MICROSECONDS);
        pogobot_timer_wait_for_expiry(timer);

        if (intensities[1] > intensities[3]) {
            // Si l'intensité vient de l'avant droit, tourner à gauche
            pogobot_motor_set(motorL, motorQuarter);
            pogobot_motor_set(motorR, motorFull);
            pogobot_led_setColor(255, 0, 0);
        } else {
            // Si l'intensité vient de l'avant gauche, tourner à droite
            pogobot_motor_set(motorL, motorFull);
            pogobot_motor_set(motorR, motorQuarter);
            pogobot_led_setColor(0, 255, 0);
        } 

        return true;
    }

    return false;
}

void user_step(void) {
    ping_robots();

    int intensities[4] = {0, 0, 0, 0};
    get_intensities(intensities);
    if (!avoid_collision(&mydata->timer_it, intensities)) {
        pogobot_led_setColor(0, 0, 255);
        pogobot_motor_set(motorL, motorStop);
        pogobot_motor_set(motorR, motorStop);
    }
}

int main(void) {
    pogobot_init();
#ifndef SIMULATOR
    printf("init ok\n");
#endif
    pogobot_start(user_init, user_step);
    return 0;
}