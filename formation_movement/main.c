// Main include for pogobots, both for real robots and for simulations
#include "pogobase.h"

#define INFRARED_POWER 3 // 1, 2, 3

// "Global" variables should be inserted within the USERDATA struct.
// /!\  In simulation, don't declare non-const global variables outside this struct, elsewise they will be shared among all agents (and this is not realistic).
typedef struct {
    // Put all global variables you want here.
    uint8_t data_foo[8];
    time_reference_t timer_it;
    int suitDeja;
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
    printf("setup ok\n");
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

    pogobot_infrared_set_power(INFRARED_POWER);
    srand(pogobot_helper_getRandSeed());

    mydata->suitDeja = 0; // au départ, le robot ne suit personne
}


// Step function. Called continuously at each step of the pogobot main loop
void user_step(void) {
    
    // Transmission d'un message
    uint8_t msg_envoi[2];
    msg_envoi[0] = 42;
    msg_envoi[1] = mydata->suitDeja; // indique si le robot suit déjà qlq (pas d'inspi pour le nom j'avoue)
    pogobot_infrared_sendLongMessage_omniSpe(msg_envoi, sizeof(msg_envoi));

    // Réception d'un message et décision du mouvement à faire
    int move_id = -1;
    pogobot_infrared_update();

    while(pogobot_infrared_message_available()>=1){
        message_t msg;
        pogobot_infrared_recover_next_message(&msg);

        // si on détecte un robot
        if(msg.header._packet_type == ir_t_user){
            //printf("Message reçu par le robot %d provenant du robot %d !\n", pogobot_helper_getid(), msg.header._sender_id);

            // si le robot actuel ne suit personne mais que l'autre si, alors notre robot le suit
            if(mydata->suitDeja < msg.payload[1]){
                pogobot_led_setColor(255, 0, 0);
                move_id = msg.header._receiver_ir_index; 
                mydata->suitDeja = 1;
            } else if(msg.payload[1] == mydata->suitDeja){ // si aucun ne suit déjà un robot (ou qu'ils suivent déjà tous deux un robot --> cas à traiter, possible pb, à voir mais là pas d'idées) 
                if(pogobot_helper_getid() < msg.header._sender_id){ // le robot avec l'id le plus petit suit l'autre
                    pogobot_led_setColor(255, 0, 0);
                    move_id = msg.header._receiver_ir_index;  
                    mydata->suitDeja = 1;
                } else {
                    pogobot_led_setColor(0, 255, 0);
                }
            } else {
                pogobot_led_setColor(0, 255, 0);
            }
        } 
        // si on détecte autre / un mur (à implémenter selon ce que va donner wall_allignment)
        else {
            printf("Mur !\n");
        }
    }

    // Mouvement 

    // s'il le détecte à droite alors il va à droite 
    // et s'il le détecte en face il tourne un peu à droite pour éviter de le foncer dedans et ensuite le suivre
    if (move_id == 1 || move_id == 0) { 
        //printf("droite\n");
        pogobot_motor_set(motorL, motorHalf);
        pogobot_motor_set(motorR, motorStop);
    } else if (move_id == 3) { // s'il le détecte à gauche alors il tourne à gauche
        //printf("gauche\n");
        pogobot_motor_set(motorL, motorStop);
        pogobot_motor_set(motorR, motorHalf);
    } else { // si on a reçu le message de derrière on l'ignore, et idem si on n'a reçu aucun message
        pogobot_motor_set(motorL, motorHalf);
        pogobot_motor_set(motorR, motorHalf);
    }
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