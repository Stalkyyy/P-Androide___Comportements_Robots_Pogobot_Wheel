
// Main include for pogobots, both for real robots and for simulations
#include "pogobase.h"
#include "motor_control.h"
#include "led_control.h"

typedef struct {
    // Put all global variables you want here.
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
}


int16_t max(int16_t a, int16_t b) {
    return a > b ? a : b;
}

int16_t max3(int16_t a, int16_t b, int16_t c) {
    return max(max(a, b), c);
}



// Step function. Called continuously at each step of the pogobot main loop
void user_step(void) {
    int16_t val[3];

    // Mesure des intensités lumineuses
    for (int i = 0; i < 3; i++) {
        val[i] = pogobot_photosensors_read(i);
    }


    /*
     * Contrôle des moteurs en fonction de la lumière
     */

    // Cas : La lumière est détectée en face du pogobot. 
    if (max3(val[0], val[1], val[2]) == max(val[0], val[1]) && abs(val[0] - val[1]) < 0.1) {
        pogobot_move_forward();
        led_set_green();
    } 
    
    // Cas : La lumière est détectée à la droite du pogobot.
    else if (max3(val[0], val[1], val[2]) == val[0]) {
        pogobot_turn_left();
        led_set_blue();
    } 
    
    // Cas : La lumière est détectée à la gauche du pogobot.
    else if (max3(val[0], val[1], val[2]) == val[1]) {
        pogobot_turn_right();
        led_set_red();
    } 

    // Cas : On tourne derrière.
    else {
        pogobot_stop();
        led_blink(255, 0, 0, 250);
    }
}



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
