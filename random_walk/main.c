#include "pogobase.h"
#include <stdlib.h>
#include <time.h>

/**
 * On suppose ici que l'utilisateur a calibré les pogobots individuellement pour la marche avant.
 * La vitesse devrait être également pris en compte pour que, le cas d'un face à face, ils puissent se détecter sans collision (probablement pour les pogobots à roues).
 * Le calibrage sera récupérable avec pogobot_motor_power_mem_get(). 
 * 
 * On différenciera les robots à roues des robots à brosses, notamment pour leur manière de tourner.
 */

#define SIMULATEUR false // Permet de choisir si on utilise le cas réel ou simulateur (pour le calibrage).

#define HAS_WHEEL true // Permet de choisir le cas où c'est un robot à roue, ou un robot à brosse.

#define NO_TURN 'N'
#define LEFT_TURN 'L'
#define RIGHT_TURN 'R'

/*
 * ====================================================================================
 */

// Prototypes des fonctions
int16_t max(int16_t a, int16_t b);
int16_t min(int16_t a, int16_t b);

void ping_robots(void);
void get_intensities(bool detection[]);

void move_front(void);
void move_left(void);
void move_right(void);
void move_stop(void);
void random_or_follow_turn(void);
void move_logic(bool *detection);

/*
 * ====================================================================================
 */


typedef struct {
    uint8_t data_foo[8];
    time_reference_t timer_it;

    uint16_t motorLeft;
    uint8_t dirLeft;

    uint16_t motorRight;
    uint8_t dirRight;

    char lastTurn;
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


/*
 * ====================================================================================
 */


void ping_robots(void) {
    uint8_t ping_message[7] = {"robots"};
    pogobot_infrared_sendLongMessage_omniSpe(ping_message, sizeof(ping_message));
}


void get_intensities(bool detection[]) {
    pogobot_infrared_update();
    while (pogobot_infrared_message_available()) {
        message_t msg;
        pogobot_infrared_recover_next_message(&msg);

        int sensor_id = msg.header._receiver_ir_index;
        if (sensor_id >= 0 && sensor_id < 4) {
            detection[sensor_id] = true;
        }
    }
}


/*
 * ====================================================================================
 */

// Si le robot a des roues, il ne change pas sa puissance, mais uniquement la direction du moteur.
// Sinon (robot à brosse), il change la puissance des moteurs.

void move_front(void) {
    if (SIMULATEUR) {
        pogobot_motor_set(motorL, motorHalf);
        pogobot_motor_set(motorR, motorHalf);
    } else if (HAS_WHEEL) {
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
    mydata->lastTurn = LEFT_TURN;

    if (SIMULATEUR) {
        pogobot_motor_set(motorL, motorStop);
        pogobot_motor_set(motorR, motorHalf);
    } else if (HAS_WHEEL) {
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
    mydata->lastTurn = RIGHT_TURN;

    if (SIMULATEUR) {
        pogobot_motor_set(motorL, motorHalf);
        pogobot_motor_set(motorR, motorStop);
    } else if (HAS_WHEEL) {
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



void random_or_follow_turn(void) {

    // Si on a déjà tourné à un côté, on prend le dernier côté effectué.
    if (mydata->lastTurn == RIGHT_TURN)
        move_right();
    else if (mydata->lastTurn == LEFT_TURN)
        move_left();

    // Sinon, on choisit un côté au hasard. 
    else if (rand() % 2) 
        move_right(); 
    else 
        move_left();
}


/*
 * ====================================================================================
 */


void move_logic(bool *detection) {
    // Initialisation des capteurs
    bool sensorFront = detection[0];
    bool sensorRight = detection[1];
    // bool sensorBack = detection[2];
    bool sensorLeft = detection[3];

    // Pour les leds
    int r = 0, g = 0, b = 0;

    if (sensorFront && sensorLeft && sensorRight) {
        if (HAS_WHEEL)
            random_or_follow_turn();
        else
            move_stop();
    }

    else if (sensorLeft && sensorRight) {
        move_front();
    } 

    else if ((sensorFront && sensorRight)) {
        move_left();
    }

    else if ((sensorFront && sensorLeft)) {
        move_right();
    }

    else if (sensorFront) {
        random_or_follow_turn();
    }
    
    else {
        move_front();
    }

    r = 100 * sensorFront;
    g = 100 * sensorRight;
    b = 100 * sensorLeft;

    pogobot_led_setColor(r, g, b);
}


/*
 * ====================================================================================
 */


void user_init(void) {
    #ifndef SIMULATOR
        printf("setup ok\n");
    #endif
        pogobot_stopwatch_reset(&mydata->timer_it);
        main_loop_hz = 45;
        max_nb_processed_msg_per_tick = 0;
        msg_rx_fn = NULL;
        msg_tx_fn = NULL;
        error_codes_led_idx = 3;
        pogobot_infrared_set_power(3);
        srand(pogobot_helper_getRandSeed());



        // Récupération des données de calibration des robots

        uint16_t power_mem[3];
        uint8_t dir_mem[3];

        pogobot_motor_power_mem_get(power_mem);
        mydata->motorLeft = power_mem[1];
        mydata->motorRight = power_mem[0];

        pogobot_motor_dir_mem_get(dir_mem);
        mydata->dirLeft = dir_mem[1];
        mydata->dirRight = dir_mem[0];

        mydata->lastTurn = NO_TURN;
    }
    
    
/*
 * ====================================================================================
 */


void user_step(void) {
    bool detection[4] = {false, false, false, false};
    ping_robots();
    get_intensities(detection);
    move_logic(detection);
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
    return 0;
}