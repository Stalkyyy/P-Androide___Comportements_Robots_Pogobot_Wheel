
// Main include for pogobots, both for real robots and for simulations
#include "pogobase.h"
#include "motor_control.h"
#include "led_control.h"

#define THRESOLD_DETECTION 10
#define THRESOLD_DIFF_FRONT 10

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

    main_loop_hz = 60;
    max_nb_processed_msg_per_tick = 0;

    msg_rx_fn = NULL;       
    msg_tx_fn = NULL;       

    error_codes_led_idx = 3;

    mydata->motorLeft = motorStop;
    mydata->motorRight = motorStop;

}


<<<<<<< HEAD
=======
int16_t max(int16_t a, int16_t b) {
    return a > b ? a : b;
}

int16_t min(int16_t a, int16_t b) {
    return a < b ? a : b;
}

int16_t max3(int16_t a, int16_t b, int16_t c) {
    return max(max(a, b), c);
}



>>>>>>> 94f3597 (Phototaxis 1.1 - mouvement)
// Step function. Called continuously at each step of the pogobot main loop
void user_step(void) {
    if (!(pogobot_ticks % 30 == 0))
        return;

    int16_t back_photo = pogobot_photosensors_read(0);
    int16_t front_left_photo = pogobot_photosensors_read(1);
    int16_t front_right_photo = pogobot_photosensors_read(2);

    int16_t max_sensor = max3(back_photo, front_left_photo, front_right_photo);
    int16_t diff_left_right = abs(front_left_photo - front_right_photo);


    /*
     * Contrôle des moteurs en fonction de la lumière
     */

    // Cas : Il n'y a pas de lumière / on considère qu'il n'y a pas de lumière.
    if (max_sensor < THRESOLD_DETECTION) {
        pogobot_stop();
        /* printf("Je ne vois rien !\n"); */
    }

    // Cas : La lumière est détectée en face du pogobot. 
<<<<<<< HEAD
    else if (max_sensor != back_photo && diff_left_right < THRESOLD_DIFF_FRONT) {
        pogobot_move_forward();
        /* printf("En face ! -> maxSensor = %d\n", max_sensor); */
    } 
    
    // Cas : La lumière est détectée à la gauche du pogobot.
    else if (max_sensor == front_left_photo) {
        pogobot_turn_left();
        /* printf("A gauche ! -> maxSensor = %d\n", max_sensor); */
    } 
    
    // Cas : La lumière est détectée à la droite du pogobot.
    else if (max_sensor == front_right_photo) {
        pogobot_turn_right();
        /* printf("A droite ! -> maxSensor = %d\n", max_sensor); */
    } 

    // Cas : La lumière est détectée à l'arrière.
    else {
        pogobot_turn_right();
        /* printf("Derrière ! -> maxSensor = %d\n", max_sensor); */
    }
=======
    if (max3(val[0], val[1], val[2]) == max(val[0], val[1]) && abs(val[0] - val[1]) < 50) {
        if (mydata->motorLeft > mydata->motorRight)
            mydata->motorRight += 3;
        else 
            mydata->motorRight += 3;

        mydata->motorLeft = max(motorHalf, mydata->motorLeft);
        mydata->motorRight = max(motorHalf, mydata->motorRight);

        // pogobot_move_forward();
        // led_set_green();
    } 
    
    // Cas : La lumière est détectée à la droite du pogobot.
    else if (max3(val[0], val[1], val[2]) == val[0]) {
        mydata->motorLeft -= 10;
        mydata->motorRight += 10;
        // pogobot_turn_left();
        // led_set_blue();
    } 
    
    // Cas : La lumière est détectée à la gauche du pogobot.
    else if (max3(val[0], val[1], val[2]) == val[1]) {
        mydata->motorLeft += 10;
        mydata->motorRight -= 10;
        // pogobot_turn_right();
        // led_set_red();
    } 

    else if (max3(val[0], val[1], val[2]) == val[2]) {
        mydata->motorLeft += 10;
        mydata->motorRight -= 10;
        // pogobot_turn_right();
        // led_set_red();
    } 

    mydata->motorLeft = max(0, mydata->motorLeft);
    mydata->motorRight = max(0, mydata->motorRight);

    mydata->motorLeft = min(motorFull, mydata->motorLeft);
    mydata->motorRight = min(motorFull, mydata->motorRight);

    pogobot_motor_set(motorR, mydata->motorRight);
    pogobot_motor_set(motorL, mydata->motorLeft);

>>>>>>> 94f3597 (Phototaxis 1.1 - mouvement)
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
