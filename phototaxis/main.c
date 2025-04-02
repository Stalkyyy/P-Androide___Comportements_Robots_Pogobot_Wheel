
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

int16_t min(int16_t a, int16_t b) {
    return a < b ? a : b;
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
 * ====================================================================================
 */



// Step function. Called continuously at each step of the pogobot main loop
void user_step(void) {
    if (pogobot_ticks % 60 != 0)
        return;

    // On récupère la valeur de la source lumineuse détectée par les sensors.
    int16_t back_photo = pogobot_photosensors_read(0);
    int16_t front_left_photo = pogobot_photosensors_read(1);
    int16_t front_right_photo = pogobot_photosensors_read(2);

    // On cherche le sensor le plus stimulé. 
    // De plus, on fait la différence entre les deux sensors du devant, afin de voir si la source lumineuse est devant ou sur les côtés.
    int16_t max_sensor = max3(back_photo, front_left_photo, front_right_photo);
    int16_t diff_left_right = abs(front_left_photo - front_right_photo);

    // Ajustement dynamique des seuils en fonction de la luminosité ambiante
    // int16_t ambient_light = (back_photo + front_left_photo + front_right_photo) / 3;
    // int16_t dynamic_threshold_min = ambient_light / 2;
    // int16_t dynamic_threshold_max = ambient_light * 2;

    printf("Capteurs - Back: %d, Front Left: %d, Front Right: %d\n", back_photo, front_left_photo, front_right_photo);

    /*
     * Contrôle des moteurs en fonction de la lumière
     */

    // Cas : "Aucune" détection de lumière ou "très proche" d'une lumière. On ralentit pour s'arrêter.
    // if (max_sensor < dynamic_threshold_min || max_sensor > dynamic_threshold_max) {
    //     mydata->motorLeft -= 5;
    //     mydata->motorRight -= 5;
    //     printf("Rien de détecté !\n");
    // }

    // Cas : La lumière est détectée en face du pogobot. 
    if (max_sensor != back_photo && diff_left_right < THRESOLD_IS_FRONT) {
        uint16_t deltaSpeed = min(100, diff_left_right) / 2;

        if (mydata->motorLeft > mydata->motorRight) {
            mydata->motorLeft += deltaSpeed;
            mydata->motorRight -= deltaSpeed;
        } else {
            mydata->motorLeft += deltaSpeed;
            mydata->motorRight -= deltaSpeed;
        }

        printf("En face !\n");
    } 
    
    // Cas : La lumière est détectée à la droite du pogobot.
    else if (max_sensor == front_right_photo) {
        mydata->motorLeft += 150;
        mydata->motorRight -= 300;

        printf("Devant droit !\n");
    } 
    
    // Cas : La lumière est détectée à la gauche du pogobot.
    else if (max_sensor == front_left_photo) {
        mydata->motorLeft -= 300;
        mydata->motorRight += 150;

        printf("Devant gauche !\n");
    } 

    // Cas : La lumière est détectée à l'arrière du pogobot.
    else if (max_sensor == back_photo) {
        mydata->motorLeft -= 300;
        mydata->motorRight += 150;

        printf("Derrière !\n");
    } 

    // Permet de pouvoir borner la vitesse des moteurs
    mydata->motorLeft = max(0, mydata->motorLeft);
    mydata->motorRight = max(0, mydata->motorRight);

    mydata->motorLeft = min(motorFull, mydata->motorLeft);
    mydata->motorRight = min(motorFull, mydata->motorRight);

    // On change la vitesse des moteurs.
    pogobot_motor_set(motorL, mydata->motorLeft);
    pogobot_motor_set(motorR, mydata->motorRight);

    printf("Left : %d, Right : %d\n", mydata->motorLeft, mydata->motorRight);

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
