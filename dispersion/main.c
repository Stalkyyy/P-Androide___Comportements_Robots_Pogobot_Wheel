
// Main include for pogobots, both for real robots and for simulations
#include "pogobase.h"

// "Global" variables should be inserted within the USERDATA struct.
// /!\  In simulation, don't declare non-const global variables outside this struct, elsewise they will be shared among all agents (and this is not realistic).
typedef struct {
    // Put all global variables you want here.
    uint8_t data_foo[8];
    time_reference_t timer_it;
    int direction_counts[4]; //Compteur de message reçus selon la direction devant, droite, derriere, gauche 
} USERDATA;

// Call this macro in the same file (.h or .c) as the declaration of USERDATA
DECLARE_USERDATA(USERDATA);

// Don't forget to call this macro in the main .c file of your project (only once!)
REGISTER_USERDATA(USERDATA);
// Now, members of the USERDATA struct can be accessed through mydata->MEMBER. E.g. mydata->data_foo
//  On real robots, the compiler will automatically optimize the code to access member variables as if they were true globals.


// Init function. Called once at the beginning of the program (cf 'pogobot_start' call in main())
void user_init(void) {
#ifndef SIMULATOR
    printf("setup ok - DISPERSION\n");
#endif

    // Init timer
    pogobot_stopwatch_reset(&mydata->timer_it);

    // Set main loop frequency, message sending frequency, message processing frequency
    main_loop_hz = 60;      // Call the 'user_step' function 60 times per second
    max_nb_processed_msg_per_tick = 0;
    // Specify functions to send/transmit messages. See the "hanabi" example to see message sending/processing in action!
    msg_rx_fn = NULL;       // If Null, no reception of message
    msg_tx_fn = NULL;       // If Null, don't send any message

    // Set led index to show error codes (e.g. time overflows)
    error_codes_led_idx = 3; // Default value, negative values to disable
    
    pogobot_infrared_set_power(3); //Puissance max pour communication InfraRouge

}

void ping_robots(void) {
    uint8_t ping_message[1] = {42}; // Mess arbitraire 
    pogobot_infrared_sendLongMessage_omniSpe(ping_message, sizeof(ping_message));
}

void update_direction_count(void){
    // Reinitialise le compteur de message
    for(int i = 0; i < 4; i++){
        mydata->direction_counts[i] = 0;
    }
    // MAJ de la file de réception InfraRouge
    pogobot_infrared_update();
    // Traitement de tous les messages disponibles
    while(pogobot_infrared_message_available()){
        message_t mess;
        pogobot_infrared_recover_next_message(&mess);

        //Récupération de la direction du capteur ayant reçu le message
        int dir = mess.header._receiver_ir_index;
        if(dir >= 0 && dir <4){
            mydata->direction_counts[dir]++; //Incremente le compteur de la direction
        }
    }
}
//Fonction de mouvement basée sur les détections pour la dispersion
void perform_dispersion_movement(void){
    int *dir_count = mydata->direction_counts;
    int tot = 0;
    for (int i =0; i <4; i++){
        tot += dir_count[i];
    }

    if (tot ==0){
        //Si aucun robot autour, On avance tout droit
        pogobot_motor_set(motorL, motorThreeQuarter);
        pogobot_motor_set(motorR, motorThreeQuarter);
        pogobot_led_setColor(0,255,0); //Vert normalement 
        return; 
    }

    //Direction la moins "encombrée"; c'est à dire la ou il faut aller ! 

    int min_dir = 0;
    for(int i = 0; i <4; i++){
        if(dir_count[i] < dir_count[min_dir]){
            min_dir = i;
        }
    }

    //Logique de mouvement basé sur les différentes directions
    switch(min_dir){
        case 0: // avant
            pogobot_motor_set(motorL,motorHalf);
            pogobot_motor_set(motorR,motorHalf);
            break;
        case 1: // droite 
            pogobot_motor_set(motorL, motorQuarter);
            pogobot_motor_set(motorR,motorStop);
            break;
        case 2: // arrière 
            pogobot_motor_dir_set(motorL, 1); // marche arrière 
            pogobot_motor_dir_set(motorR, 1);
            pogobot_motor_set(motorL, motorHalf);
            pogobot_motor_set(motorR, motorHalf);
            break;   
        case 3: // gauche
            pogobot_motor_set(motorL, motorStop);
            pogobot_motor_set(motorR, motorQuarter);
            break;
    }

    //LED Orange, cela veut dire que c'est en etat de dispersion
    pogobot_led_setColor(255,128,0);
}

// Step function. Called continuously at each step of the pogobot main loop
void user_step(void) {
  ping_robots();  //Envoi un ping InfraRouge
  update_direction_count(); // Analyse les Messages InfraRouge reçus
  perform_dispersion_movement(); // Bouge en Direction opposée aux autres 
}


// Entrypoint of the program
int main(void) {
    pogobot_init();     // Initialization routine for the robots
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
