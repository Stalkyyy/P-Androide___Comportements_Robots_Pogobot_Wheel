
// Main include for pogobots, both for real robots and for simulations
#include "pogobase.h"
#include <stdlib.h>
#include <math.h>
#ifndef M_PI_4 // Car parfois il n'est pas inclus dans math.h
#define M_PI_4 0.78539816339744830962 // Valeur de PI/4
#endif

// "Global" variables should be inserted within the USERDATA struct.
// /!\  In simulation, don't declare non-const global variables outside this struct, elsewise they will be shared among all agents (and this is not realistic).

//Définition de Variables Globales
#define DETECTION_TIMEOUT_MS 4000 // Temps d'attente avant de considérer qu'il n'y a plus de robots
#define DISPERSION_INERTIE_TICKS 30 // Nombre de ticks d'inertie avant de changer de direction
typedef struct {
    // Put all global variables you want here.
    uint8_t data_foo[8];
    time_reference_t timer_it;
    int direction_counts[4]; //Compteur de message reçus selon la direction devant, droite, derriere, gauche 

    uint32_t last_detect; // 
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
    
    pogobot_infrared_set_power(1); //Puissance pour communication InfraRouge

    //Initialisation des variables
    srand(pogobot_helper_getRandSeed()); // Genere une seed aléatoire avec la tension de la batterie
    mydata->last_detect = current_time_milliseconds(); // Démarre immédiatement 

}

void ping_robots(void) {
    uint8_t ping_message[1] = {rand()%255}; // Mess avcec val aléatoire pour que ca parte dans toute les directions
    pogobot_infrared_sendLongMessage_omniSpe(ping_message, sizeof(ping_message));
    
}
void ping_wall(void) {
    uint8_t wall_message[1] = {99}; // Message spécifique pour représenter un mur
    pogobot_infrared_sendLongMessage_omniSpe(wall_message, sizeof(wall_message));
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


         /*Vérifie si le messsage est un mur 
         if (mess.payload[0] == 99){
            printf("MUR DETECTE\n");
            mydata->direction_counts[0] = 1; // Dit que le mur est devant
            continue;
        } */

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
    //Calcul du nombre tot de robots vus 
    for (int i =0; i <4; i++){
        tot += dir_count[i];
    }
    uint32_t now = current_time_milliseconds(); // Temps actuel
    int x;
    int y;
    //Vecteurs unitaire par message recus
    x += dir_count[1]; //droite 
    x -= dir_count[3]; //gauche
    y += dir_count[0]; //avant$
    y -= dir_count[2]; //arrière
    
    //calcul du vect opposé pour partir
    int dx = -x;
    int dy = -y;

    if (tot > 0){
        mydata->last_detect = now; // On a detecter un robot, on reset le timer
    }


    if (tot == 0){
        int dir = rand()% 4;
        switch(dir){
            case 0:
                pogobot_motor_set(motorL, motorHalf);
                pogobot_motor_set(motorR, motorHalf);
                break;
            case 1:
                pogobot_motor_set(motorL, motorHalf);
                pogobot_motor_set(motorR, motorStop);
                break;
            case 2:
                pogobot_motor_set(motorL, -motorThreeQuarter);
                pogobot_motor_set(motorR, -motorThreeQuarter);
                break;
            case 3:
                pogobot_motor_set(motorL, motorStop);
                pogobot_motor_set(motorR, motorHalf);
                break;
        }
        pogobot_led_setColor(0,0,255); //Bleu normalement   
        return;
    }
    

    float angle = atan2(dy, dx); // Calcul de l'angle opposé

    if (angle > -M_PI_4 && angle <= M_PI_4){
        // On avance
        pogobot_motor_set(motorL, motorThreeQuarter);
        pogobot_motor_set(motorR, motorThreeQuarter);
    } else if (angle > M_PI_4 && angle <= 3*M_PI_4){
        // On tourne à gauche
        pogobot_motor_set(motorL, motorStop);
        pogobot_motor_set(motorR, motorThreeQuarter);
    } else if (angle < -M_PI_4 && angle >= -3*M_PI_4){
        // On tourne à droite
        pogobot_motor_set(motorL, motorThreeQuarter);
        pogobot_motor_set(motorR, motorStop);
        
    } else {
        // On recule
        pogobot_motor_set(motorL, -motorThreeQuarter);
        pogobot_motor_set(motorR, -motorThreeQuarter);
        
    }
    pogobot_let_setColor(255,128,0); //Orange normalement
}

// Step function. Called continuously at each step of the pogobot main loop
void user_step(void) {
  //ping_wall(); // Envoi un ping InfraRouge pour détecter les murs
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
