
// Main include for pogobots, both for real robots and for simulations
#include "pogobase.h"

#define THRESOLD_IS_FRONT 15

#define HAS_WHEEL true // Permet de choisir le cas où c'est un robot à roue, ou un robot à brosse.
#define NO_TURN 'N'
#define LEFT_TURN 'L'
#define RIGHT_TURN 'R'


typedef struct {
    // Put all global variables you want here.
    uint8_t data_foo[8];
    time_reference_t timer_it;

    uint16_t motorLeft;
    uint8_t dirLeft;

    uint16_t motorRight;
    uint8_t dirRight;

} USERDATA;

DECLARE_USERDATA(USERDATA);
REGISTER_USERDATA(USERDATA);




/*
 * ====================================================================================
 */


// Prototypes des fonctions
int16_t max(int16_t a, int16_t b);
int16_t max3(int16_t a, int16_t b, int16_t c);
int16_t min(int16_t a, int16_t b);

void move_front(void);
void move_left(uint16_t power);
void move_right(uint16_t power);
void move_stop(void);


/*
 * ====================================================================================
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

/*
 * ====================================================================================
 */


// Si le robot a des roues, il ne change pas sa puissance, mais uniquement la direction du moteur.
// Sinon (robot à brosse), il change la puissance des moteurs.

void move_front(void) {
    if (HAS_WHEEL) {
        pogobot_motor_set(motorL, mydata->motorLeft);
        pogobot_motor_set(motorR, mydata->motorRight);

        pogobot_motor_dir_set(motorL, mydata->dirLeft);
        pogobot_motor_dir_set(motorR, mydata->dirRight);
    } else {
        pogobot_motor_set(motorL, mydata->motorLeft);
        pogobot_motor_set(motorR, mydata->motorRight);
    }
}

void move_left(uint16_t power) {
    if (HAS_WHEEL) {
        pogobot_motor_set(motorL, power);
        pogobot_motor_set(motorR, power);

        pogobot_motor_dir_set(motorL, (mydata->dirLeft + 1) % 2);
        pogobot_motor_dir_set(motorR, mydata->dirRight);
    } else {
        pogobot_motor_set(motorL, motorStop);
        pogobot_motor_set(motorR, power);
    }
}

void move_right(uint16_t power) {
    if (HAS_WHEEL) {
        pogobot_motor_set(motorL, power);
        pogobot_motor_set(motorR, power);

        pogobot_motor_dir_set(motorL, mydata->dirLeft);
        pogobot_motor_dir_set(motorR, (mydata->dirRight + 1) % 2);
    } else {
        pogobot_motor_set(motorL, motorStop);
        pogobot_motor_set(motorR, power);
    }
}

void move_stop(void) {
    pogobot_motor_set(motorL, motorStop);
    pogobot_motor_set(motorR, motorStop);
}


/*
 * ====================================================================================
 */


void user_init(void) {
    #ifndef SIMULATOR
        printf("setup ok\n");
    #endif
        pogobot_stopwatch_reset(&mydata->timer_it);
    
        main_loop_hz = 10; // Call the 'user_step' function 10 times per second
        max_nb_processed_msg_per_tick = 0;
    
        msg_rx_fn = NULL;       
        msg_tx_fn = NULL;       
    
        error_codes_led_idx = 3;
    
        // Récupération des données de calibration des robots

        uint16_t power_mem[3];
        uint8_t dir_mem[3];

        pogobot_motor_power_mem_get(power_mem);
        mydata->motorLeft = power_mem[1];
        mydata->motorRight = power_mem[0];

        pogobot_motor_dir_mem_get(dir_mem);
        mydata->dirLeft = dir_mem[1];
        mydata->dirRight = dir_mem[0];    
    }


/*
 * ====================================================================================
 */



void user_step(void) {

    // On récupère la valeur de la source lumineuse détectée par les sensors.
    int16_t back_photo = pogobot_photosensors_read(0);
    int16_t front_left_photo = pogobot_photosensors_read(1);
    int16_t front_right_photo = pogobot_photosensors_read(2);

    // On cherche le sensor le plus stimulé. On fait la différence entre les deux sensors du devant, afin de voir si la source lumineuse est devant ou sur les côtés.
    int16_t max_sensor = max3(back_photo, front_left_photo, front_right_photo);
    int16_t diff_left_right = abs(front_left_photo - front_right_photo);


    /*
     * Contrôle des moteurs en fonction de la lumière
     */

    // Cas : La lumière est détectée en face du pogobot. 
    if (max_sensor != back_photo && diff_left_right < THRESOLD_IS_FRONT) {
        move_front();
    } 
    
    // Cas : La lumière est détectée à la droite du pogobot.
    else if (max_sensor == front_right_photo) {
        move_right(motorQuarter);
    } 
    
    // Cas : La lumière est détectée à la gauche du pogobot.
    else if (max_sensor == front_left_photo) {
        move_left(motorQuarter);
    } 

    // Cas : La lumière est détectée à l'arrière du pogobot.
    else if (max_sensor == back_photo) {
        // if (max(front_left_photo, front_right_photo) == front_left_photo)
        //     move_left(motorQuarter);
        // else
        //     move_right(motorQuarter);

        move_right(motorQuarter);
    } 
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
