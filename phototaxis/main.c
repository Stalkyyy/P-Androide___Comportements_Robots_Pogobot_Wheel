
// Main include for pogobots, both for real robots and for simulations
#include "pogobase.h"

#define THRESOLD_MIN_DETECTION 10
#define THRESOLD_MAX_DETECTION 10

#define THRESOLD_IS_FRONT 10

typedef struct {
    // Put all global variables you want here.
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

int16_t max3(int16_t a, int16_t b, int16_t c) {
    return max(max(a, b), c);
}

/*
 * ====================================================================================
 */


void user_init(void) {
#ifndef SIMULATOR
    printf("setup ok\n");
#endif
    pogobot_stopwatch_reset(&mydata->timer_it);

    main_loop_hz = 60; // Call the 'user_step' function 60 times per second
    max_nb_processed_msg_per_tick = 0;

    msg_rx_fn = NULL;       
    msg_tx_fn = NULL;       

    error_codes_led_idx = 3;

    mydata->motorLeft = motorStop;
    mydata->motorRight = motorStop;

}

/*
 * utils
 */

int16_t max(int16_t a, int16_t b) {
    return a > b ? a : b;
}

int16_t min(int16_t a, int16_t b) {
    return a < b ? a : b;
}

int16_t max3(int16_t a, int16_t b, int16_t c) {
    return max(max(a, b), c);
}



// Step function. Called continuously at each step of the pogobot main loop
void user_step(void) {
    if (pogobot_ticks % 60 != 0)
        return;

    int16_t back_photo = pogobot_photosensors_read(0);
    int16_t front_left_photo = pogobot_photosensors_read(1);
    int16_t front_right_photo = pogobot_photosensors_read(2);

    int16_t max_sensor = max3(back_photo, front_left_photo, front_right_photo);
    int16_t diff_left_right = abs(front_left_photo - front_right_photo);


    /*
     * Contrôle des moteurs en fonction de la lumière
     */

    // Cas : "Aucune" détection de lumière ou "très proche" d'une lumière. On ralentit pour s'arrêter.
    if (max_sensor < THRESOLD_MIN_DETECTION || max_sensor > THRESOLD_MAX_DETECTION) {
        mydata->motorLeft -= 10;
        mydata->motorRight -= 10;
    }

    // Cas : La lumière est détectée en face du pogobot. 
    else if (max_sensor != back_photo && diff_left_right < THRESOLD_IS_FRONT) {
        uint16_t deltaSpeed = min(20, diff_left_right) / 2;

        if (mydata->motorLeft > mydata->motorRight) {
            mydata->motorLeft += deltaSpeed;
            mydata->motorRight -= deltaSpeed;
        } else {
            mydata->motorLeft += deltaSpeed;
            mydata->motorRight -= deltaSpeed;
        }
    } 
    
    // Cas : La lumière est détectée à la droite du pogobot.
    else if (max_sensor == front_right_photo) {
        mydata->motorLeft += 10;
        mydata->motorRight -= 10;
    } 
    
    // Cas : La lumière est détectée à la gauche du pogobot.
    else if (max_sensor == front_left_photo) {
        mydata->motorLeft -= 10;
        mydata->motorRight += 10;
    } 

    // Cas : La lumière est détectée à l'arrière du pogobot.
    else if (max_sensor == back_photo) {
        mydata->motorLeft -= 10;
        mydata->motorRight += 10;
    } 

    // Permet de pouvoir borner la vitesse des moteurs
    mydata->motorLeft = max(0, mydata->motorLeft);
    mydata->motorRight = max(0, mydata->motorRight);

    mydata->motorLeft = min(motorFull, mydata->motorLeft);
    mydata->motorRight = min(motorFull, mydata->motorRight);

    // On change la vitesse des moteurs.
    pogobot_motor_set(motorR, mydata->motorRight);
    pogobot_motor_set(motorL, mydata->motorLeft);

}


/*
 * ====================================================================================
 */


// Entrypoint of the program
int main(void) {
    pogobot_init();     
#ifndef SIMULATOR
    printf("init ok\n");
#endif

    // Specify the user_init and user_step functions
    pogobot_start(user_init, user_step);
    return 0;
}



// MODELINE "{{{1
// vim:expandtab:softtabstop=4:shiftwidth=4:fileencoding=utf-8
// vim:foldmethod=marker
