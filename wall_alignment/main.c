// Main include for pogobots, both for real robots and for simulations
#include "pogobase.h"

#define INFRARED_POWER 3 // 1, 2, 3

#define HAS_WHEEL false // Permet de choisir le cas où c'est un robot à roue, ou un robot à brosse.

// "Global" variables should be inserted within the USERDATA struct.
// /!\  In simulation, don't declare non-const global variables outside this struct, elsewise they will be shared among all agents (and this is not realistic).
typedef struct {
    // Put all global variables you want here.
    uint8_t data_foo[8];
    time_reference_t timer_it;

    uint16_t motorLeft;
    uint8_t dirLeft;

    uint16_t motorRight;
    uint8_t dirRight;

    time_reference_t waiting_time;
} USERDATA;

// Call this macro in the same file (.h or .c) as the declaration of USERDATA
DECLARE_USERDATA(USERDATA);

// Don't forget to call this macro in the main .c file of your project (only once!)
REGISTER_USERDATA(USERDATA);
// Now, members of the USERDATA struct can be accessed through mydata->MEMBER. E.g. mydata->data_foo
//  On real robots, the compiler will automatically optimize the code to access member variables as if they were true globals.


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

void move_left(void) {
    if (HAS_WHEEL) {
        pogobot_motor_set(motorL, motorHalf);
        pogobot_motor_set(motorR, motorHalf);

        pogobot_motor_dir_set(motorL, (mydata->dirLeft + 1 % 2));
        pogobot_motor_dir_set(motorR, mydata->dirRight);
    } else {
        pogobot_motor_set(motorL, motorStop);
        pogobot_motor_set(motorR, motorHalf);
    }
}

void move_right(void) {
    if (HAS_WHEEL) {
        pogobot_motor_set(motorL, motorHalf);
        pogobot_motor_set(motorR, motorHalf);

        pogobot_motor_dir_set(motorL, mydata->dirLeft);
        pogobot_motor_dir_set(motorR, (mydata->dirRight + 1 % 2));
    } else {
        pogobot_motor_set(motorL, motorHalf);
        pogobot_motor_set(motorR, motorStop);
    }
}

void move_stop(void) {
    pogobot_motor_set(motorL, motorStop);
    pogobot_motor_set(motorR, motorStop);
}



// Init function. Called once at the beginning of the program (cf 'pogobot_start' call in main())
void user_init(void) {
#ifndef SIMULATOR
    printf("setup ok\n");
#endif

    // Init timer
    pogobot_stopwatch_reset(&mydata->timer_it);

    // Set main loop frequency, message sending frequency, message processing frequency
    main_loop_hz = 30;      // Call the 'user_step' function 60 times per second
    max_nb_processed_msg_per_tick = 0;
    // Specify functions to send/transmit messages. See the "hanabi" example to see message sending/processing in action!
    msg_rx_fn = NULL;       // If Null, no reception of message
    msg_tx_fn = NULL;       // If Null, don't send any message

    // Set led index to show error codes (e.g. time overflows)
    error_codes_led_idx = 3; // Default value, negative values to disable

    pogobot_infrared_set_power(INFRARED_POWER);
    srand(pogobot_helper_getRandSeed()); // Initialiser le générateur de nombres aléatoires

    // Récupération des données de calibration des robots

    uint16_t power_mem[3];
    uint8_t dir_mem[3];

    if(HAS_WHEEL){
        pogobot_motor_power_mem_get(power_mem);
        mydata->motorLeft = power_mem[1];
        mydata->motorRight = power_mem[0];

        pogobot_motor_dir_mem_get(dir_mem);
        mydata->dirLeft = dir_mem[1];
        mydata->dirRight = dir_mem[0];
    } else {
        mydata->motorLeft = motorHalf;
        mydata->motorRight = motorHalf;
    }

}


// Step function. Called continuously at each step of the pogobot main loop
void user_step(void) {

    // Début du temps d'attente 
    pogobot_stopwatch_reset(&mydata->waiting_time);

    // Transmission d'un message
    uint8_t msg_envoi[4];
    msg_envoi[1] = 42;
    pogobot_infrared_sendLongMessage_omniSpe(msg_envoi, sizeof(msg_envoi));

    // Réception d'un message
    uint8_t move_id = 5;
    pogobot_infrared_update();
    uint32_t end_waiting_time = 0; // le temps mis pour envoyer et recevoir une détection du mur

    while(pogobot_infrared_message_available() >= 1){
        message_t msg;
        pogobot_infrared_recover_next_message(&msg);

        // si mur détecté
        if(msg.header._packet_type == ir_t_user){
            move_id = msg.header._receiver_ir_index;
            //end_waiting_time += pogobot_stopwatch_get_elapsed_microseconds(&mydata->waiting_time);
            end_waiting_time += pogobot_stopwatch_get_elapsed_microseconds(&mydata->waiting_time);
        } else { // c'est un robot et qu'il se trouve devant nous
            if(msg.header._receiver_ir_index == 0 && msg.header._sender_ir_index == 2){
                move_id = 1; // on change de direction pour ne plus être derrière lui
            }
        }
    }

    // si aucune détection de mur
    if(end_waiting_time == 0){
        end_waiting_time = 1000;
    }

    printf("Temps mis pour détecter le mur : %d micro secondes\n", end_waiting_time);

    // Mouvement

    // si on a reçu la détection du mur rapidement (temps à revoir selon les tests)
    // alors on s'arrête devant le mur
    if(end_waiting_time <= 200) { // en microseconds
        move_stop();
        pogobot_led_setColor(255, 0, 0);
    } else {
        pogobot_led_setColor(0, 255, 0);
        if (move_id == 1 || move_id == 2) { // au cas où le mur est derrière on commence à tourner
            move_right();
        } else if (move_id == 3) {
            move_left();
        } else { // si on a reçu le message d'en face on avance, et idem si on n'a reçu aucun message
            move_front();
        }
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