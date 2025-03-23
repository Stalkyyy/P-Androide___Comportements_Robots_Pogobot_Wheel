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

void get_intensities(bool detection[]) {
    pogobot_infrared_update();
    while (pogobot_infrared_message_available()) {
        message_t msg;
        pogobot_infrared_recover_next_message(&msg);

        int sensor_id = msg.header._receiver_ir_index;
        if (sensor_id >= 0 && sensor_id < 4) {
            detection[sensor_id] = true;
        }
    }
}



void movement(bool *detection) {
    // Initialisation des capteurs
    bool sensorFront = detection[0];
    bool sensorRight = detection[1];
    bool sensorBack = detection[2];
    bool sensorLeft = detection[3];

    // Initialisation des moteurs
    uint16_t motorLeft = motorThreeQuarter;
    uint16_t motorRight = motorThreeQuarter;

    // Pour les leds
    int r = 0, g = 0, b = 0;


    // Gestion des obstacles
    if (sensorBack) {
        motorLeft += motorQuarter;
        motorRight += motorQuarter;
    }

    if (sensorFront) {
        motorLeft -= motorQuarter;
        motorRight -= motorQuarter;
    }

    if (sensorRight) {
        motorRight -= motorHalf;
    }

    if (sensorLeft) {
        motorLeft -= motorHalf;
    }

    // Limitation des moteurs dans la plage
    // motorLeft = (motorLeft > motorFull) ? motorFull : (motorLeft < motorStop ? motorStop : motorLeft);
    // motorRight = (motorRight > motorFull) ? motorFull : (motorRight < motorStop ? motorStop : motorRight);


    // Les couleurs des leds
    if (motorLeft == motorRight) {
        // Avance ou recule en ligne droite
        if (motorLeft > motorThreeQuarter) {
            g = 255;  // Vert pour avancer
        } else if (motorLeft < motorQuarter) {
            r = 255;  // Rouge pour reculer
        } else {
            g = 128;  // Vert clair pour vitesse moyenne
        }
    } else if (motorLeft > motorRight) {
        // Tourne à droite
        b = 255;  // Bleu pour tourner à gauche
    } else if (motorRight > motorLeft) {
        // Tourne à gauche
        r = 255;
        g = 255;  // Jaune pour tourner à droite
    }

    // Si très lent ou à l'arrêt, lumière blanche
    if (motorLeft == motorStop && motorRight == motorStop) {
        r = g = b = 128;  // Blanc/gris clair pour l'arrêt
    }

    pogobot_led_setColor(r, g, b);

    // Commande des moteurs
    pogobot_motor_set(motorL, motorLeft);
    pogobot_motor_set(motorR, motorRight);
    //printf("Left : %d <===> Right : %d\n", motorLeft, motorRight);
}

void user_step(void) {
    ping_robots();
    bool detection[4] = {false, false, false, false};
    get_intensities(detection);
    movement(detection);
}

int main(void) {
    pogobot_init();
#ifndef SIMULATOR
    printf("init ok\n");
#endif
    pogobot_start(user_init, user_step);
    return 0;
}