#include "pogobase.h"
#include <stdlib.h>
#include <time.h>

#define MIN_DISTANCE 0
#define ACTION_DELAY_MICROSECONDS 16

typedef struct {
    uint8_t data_foo[8];
    time_reference_t timer_it;
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
    bool sensorBack = detection[2];
    bool sensorLeft = detection[3];

    // Pour les leds
    int r = 0, g = 0, b = 0;


    if (sensorFront) {
        mydata->motorLeft = motorStop;
        mydata->motorRight = motorHalf;
        r = 255;
    } 
    
    else if (sensorBack) {
        mydata->motorLeft = motorFull;
        mydata->motorRight = motorFull;
        b = 255;
    } 
    
    else if (sensorRight) {
        // Obstacle à droite : tourner à gauche
        mydata->motorLeft = motorQuarter;
        mydata->motorRight = motorHalf;
        g = 255; 
    } 
    
    else if (sensorLeft) {
        mydata->motorLeft = motorHalf;
        mydata->motorRight = motorQuarter;
        g = 255; 
    } 
    
    else {
        mydata->motorLeft = motorHalf;
        mydata->motorRight = motorHalf;
        r = 0; g = 0; b = 0;
    }

    // Permet de pouvoir borner la vitesse des moteurs
    // mydata->motorLeft = max(0, mydata->motorLeft);
    // mydata->motorRight = max(0, mydata->motorRight);

    // mydata->motorLeft = min(motorFull, mydata->motorLeft);
    // mydata->motorRight = min(motorFull, mydata->motorRight);


    r = 100 * sensorFront;
    g = 100 * sensorRight;
    b = 100 * sensorLeft;


    pogobot_led_setColor(r, g, b);
    pogobot_motor_set(motorL, mydata->motorLeft);
    pogobot_motor_set(motorR, mydata->motorRight);
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

        // Initialisation des moteurs
        // mydata->motorLeft = motorHalf;
        // mydata->motorRight = motorHalf;
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