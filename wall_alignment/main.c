#include "pogobase.h"
#include <string.h>

/**
 * On suppose ici que l'utilisateur a calibré les pogobots individuellement pour la marche avant.
 * Suivant la vitesse lors du calibrage, il faudrait changer MICRO_SECS_TIMER pour "éviter une collision". 
 * Le calibrage sera récupérable avec pogobot_motor_power_mem_get(). 
 * 
 * On différenciera les robots à roues des robots à brosses, notamment pour leur manière de tourner.
 * 
 * On suppose qu'une fois qu'on est arrêté, le robot ne sera pas déplacé.
 */


#define HAS_WHEEL true              // Permet de choisir le cas où c'est un robot à roue, ou un robot à brosse.
#define MICRO_SECS_TIMER 650000     // Le temps (en microsecondes) pour l'expiration du timer.


/*
 * ====================================================================================
 */

// Prototypes des fonctions
void move_front(void);
void move_left(void);
void move_right(void);
void move_back(void);
void move_stop(void);

bool ping_walls(void);
void walls_user_init(void);
void walls_user_step(void);

/*
 * ====================================================================================
 */


typedef struct {
    time_reference_t wall_timer;
    bool wall_detected;

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


// Si le robot a des roues, il ne change pas sa puissance, mais uniquement la direction du moteur.
// Sinon (robot à brosse), il change la puissance des moteurs.

void move_front(void) {
#ifdef SIMULATOR
    pogobot_motor_set(motorL, motorHalf);
    pogobot_motor_set(motorR, motorHalf);

    pogobot_motor_dir_set(motorL, 1);
    pogobot_motor_dir_set(motorR, 0);
#else
    if (HAS_WHEEL) {
        pogobot_motor_set(motorL, mydata->motorLeft);
        pogobot_motor_set(motorR, mydata->motorRight);

        pogobot_motor_dir_set(motorL, mydata->dirLeft);
        pogobot_motor_dir_set(motorR, mydata->dirRight);
    } else {
        pogobot_motor_set(motorL, mydata->motorLeft);
        pogobot_motor_set(motorR, mydata->motorRight);
    }
#endif
}

// Seulement pour les pogobots à roues.
void move_back(void) {
#ifdef SIMULATOR
    pogobot_motor_set(motorL, motorHalf);
    pogobot_motor_set(motorR, motorHalf);

    pogobot_motor_dir_set(motorL, 0);
    pogobot_motor_dir_set(motorR, 1);
#else
    if (HAS_WHEEL) {

        pogobot_motor_set(motorL, mydata->motorLeft);
        pogobot_motor_set(motorR, mydata->motorRight);

        pogobot_motor_dir_set(motorL, (mydata->dirLeft + 1) % 2);
        pogobot_motor_dir_set(motorR, (mydata->dirRight + 1) % 2);
    }
#endif
}

void move_left(void) {
#ifdef SIMULATOR
    pogobot_motor_set(motorL, motorHalf);
    pogobot_motor_set(motorR, motorHalf);

    pogobot_motor_dir_set(motorL, 0);
    pogobot_motor_dir_set(motorR, 0);
#else
    if (HAS_WHEEL) {
        pogobot_motor_set(motorL, motorHalf);
        pogobot_motor_set(motorR, motorHalf);

        pogobot_motor_dir_set(motorL, (mydata->dirLeft + 1) % 2);
        pogobot_motor_dir_set(motorR, mydata->dirRight);
    } else {
        pogobot_motor_set(motorL, motorStop);
        pogobot_motor_set(motorR, motorHalf);
    }
#endif
}

void move_right(void) {
#ifdef SIMULATOR
    pogobot_motor_set(motorL, motorHalf);
    pogobot_motor_set(motorR, motorHalf);

    pogobot_motor_dir_set(motorL, 1);
    pogobot_motor_dir_set(motorR, 1);
#else
    if (HAS_WHEEL) {
        pogobot_motor_set(motorL, motorHalf);
        pogobot_motor_set(motorR, motorHalf);

        pogobot_motor_dir_set(motorL, mydata->dirLeft);
        pogobot_motor_dir_set(motorR, (mydata->dirRight + 1 % 2));
    } else {
        pogobot_motor_set(motorL, motorHalf);
        pogobot_motor_set(motorR, motorStop);
    }
#endif
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

    main_loop_hz = 30;
    max_nb_processed_msg_per_tick = 0;
    msg_rx_fn = NULL;
    msg_tx_fn = NULL;
    error_codes_led_idx = 3;
    pogobot_infrared_set_power(3);
    srand(pogobot_helper_getRandSeed());

    pogobot_stopwatch_reset(&mydata->wall_timer);
    mydata->wall_detected = false;
    


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

    // Transmission d'un message pour différencier les robots des murs.
    // uint8_t msg_robot[7] = "robots";
    // pogobot_infrared_sendLongMessage_omniSpe(msg_robot, sizeof(msg_robot));

    pogobot_infrared_update();
    while(pogobot_infrared_message_available()){

        message_t msg;
        pogobot_infrared_recover_next_message(&msg);

        // Cas où le mur est détecté.
        uint8_t wall_msg[5] = "wall";
        if(memcmp(wall_msg, msg.payload, sizeof(wall_msg)) == 0) {

            // =====================================================================

            // Si on a jamais détecté un mur, alors on lance le timer d'arrêt.
            if(!mydata->wall_detected){
                mydata->wall_detected = true;
                printf("DETECTED\n");
                pogobot_timer_init(&mydata->wall_timer, MICRO_SECS_TIMER);
            }

            // =====================================================================

            // Si le timer a expiré, alors le robot s'arrête.
            if(pogobot_timer_has_expired(&mydata->wall_timer)) {
                pogobot_led_setColor(255, 0, 0);
                move_stop();
            } 
            
            // Sinon, il va tourner jusqu'à qu'il soit dos au mur, et reculer. Si on a des brosses, on avance contre le mur.
            else {
                pogobot_led_setColor(0, 255, 0);

                #ifdef SIMULATOR
                    // Robot à roues.
                #else
                if (!HAS_WHEEL) {
                    move_front();
                    return;
                }
                #endif

                // Robots à roues
                switch (msg.header._receiver_ir_index) {
                    case 2 :
                        move_back();
                        break;
                    case 3 :
                        move_right();
                        break;
                    default:
                        move_left();
                        break;
                }
            }
        } 
        
        // Cas où un robot est détecté.
        // else { 
        //     if (msg.header._receiver_ir_index == ir_front)
        //         move_right();
        // }
    }

    // Si on a détecté aucun mur, alors on avance tout droit.
    if(mydata->wall_detected == 0){
        move_front();
    }
}


/*
 * ====================================================================================
 */


bool ping_walls(void) {
    uint8_t ping_message[5] = {"wall"};
    return pogobot_infrared_sendLongMessage_omniSpe(ping_message, sizeof(ping_message));
}

void walls_user_init(void) {
#ifndef SIMULATOR
    printf("setup ok\n");
#endif
    // Initialize the random number generator
    srand(pogobot_helper_getRandSeed());
    pogobot_infrared_set_power(3);

    main_loop_hz = 45;
    max_nb_processed_msg_per_tick = 0;
    percent_msgs_sent_per_ticks = 100;
    msg_rx_fn = NULL;
    msg_tx_fn = ping_walls;
    error_codes_led_idx = -1;
}

void walls_user_step(void) {
    // ...
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
    pogobot_start(walls_user_init, walls_user_step, "walls");
    return 0;
}