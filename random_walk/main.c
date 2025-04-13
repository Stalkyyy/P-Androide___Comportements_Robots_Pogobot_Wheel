#include "pogobase.h"
#include <stdlib.h>
#include <time.h>

/**
 * On suppose ici que l'utilisateur a calibré les pogobots individuellement pour la marche avant.
 * La vitesse devrait être également pris en compte pour que, le cas d'un face à face, ils puissent se détecter sans collision (probablement pour les pogobots à roues).
 * Le calibrage sera récupérable avec pogobot_motor_power_mem_get(). 
 */

/*
 * ====================================================================================
 */

// Prototypes des fonctions
int16_t max(int16_t a, int16_t b);
int16_t min(int16_t a, int16_t b);
void ping_robots(void);
void get_intensities(bool detection[]);
void movement(bool *detection);

/*
 * ====================================================================================
 */


typedef struct {
    uint8_t data_foo[8];
    time_reference_t timer_it;

    bool is_stabilized;

    uint16_t motorLeft;
    uint16_t motorRight;
} USERDATA;

DECLARE_USERDATA(USERDATA);
REGISTER_USERDATA(USERDATA);


/*
 * ====================================================================================
 */


 int16_t max(int16_t a, int16_t b) {
    return a > b ? a : b;
}

int16_t min(int16_t a, int16_t b) {
    return a < b ? a : b;
}


/*
 * ====================================================================================
 */


void ping_robots(void) {
    uint8_t ping_message[7] = {"robots"};
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


/*
 * ====================================================================================
 */


void movement(bool *detection) {
    // Initialisation des capteurs
    bool sensorFront = detection[0];
    bool sensorRight = detection[1];
    // bool sensorBack = detection[2];
    bool sensorLeft = detection[3];

    // Pour les leds
    int r = 0, g = 0, b = 0;

    if (sensorFront && sensorLeft && sensorRight) {
        pogobot_motor_set(motorL, motorStop);
        pogobot_motor_set(motorR, motorStop);
    }

    else if (sensorLeft && sensorRight) {
        pogobot_motor_set(motorL, motorHalf);
        pogobot_motor_set(motorR, motorStop);
    } 

    else if ((sensorFront && sensorRight)) {
        pogobot_motor_set(motorL, motorStop);
        pogobot_motor_set(motorR, motorHalf);
    }

    else if ((sensorFront && sensorLeft) || sensorFront) {
        pogobot_motor_set(motorL, motorHalf);
        pogobot_motor_set(motorR, motorStop);
    }
    
    else {
        pogobot_motor_set(motorL, mydata->motorLeft);
        pogobot_motor_set(motorR, mydata->motorRight);
    }

    r = 100 * sensorFront;
    g = 100 * sensorRight;
    b = 100 * sensorLeft;

    pogobot_led_setColor(r, g, b);
}


/*
 * ====================================================================================
 */


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

        uint16_t power_mem[3];
        pogobot_motor_power_mem_get(power_mem);

        mydata->motorLeft = power_mem[1];
        mydata->motorRight = power_mem[0];
    }
    
    
/*
 * ====================================================================================
 */


void user_step(void) {
    if (pogobot_ticks % 6 != 0)
        return;

    bool detection[4] = {false, false, false, false};

    ping_robots();
    get_intensities(detection);
    movement(detection);
}


/*
 * ====================================================================================
 */


int main(void) {
    pogobot_init();
#ifndef SIMULATOR
    printf("init ok\n");
#endif
    pogobot_start(user_init, user_step);
    return 0;
}