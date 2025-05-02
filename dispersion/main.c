#include "pogobase.h"
#include <stdlib.h>
#include <time.h>

#define HAS_WHEEL true // Utiliser les réglages spécifiques pour les robots à roues
#define motorSpeed 1023 // Puissance maximale pour les moteurs

#define NO_TURN 'N'
#define LEFT_TURN 'L'
#define RIGHT_TURN 'R'

// Structure des données utilisateur, utilisée pour stocker l'état du robot
typedef struct {
    time_reference_t timer_it; // Chronomètre interne
    uint16_t motorLeft;        // Vitesse calibrée du moteur gauche
    uint8_t dirLeft;           // Direction du moteur gauche
    uint16_t motorRight;       // Vitesse calibrée du moteur droit
    uint8_t dirRight;          // Direction du moteur droit
    char lastTurn;             // Dernier virage effectué
} USERDATA;

DECLARE_USERDATA(USERDATA);
REGISTER_USERDATA(USERDATA);

// Envoie un message IR pour que les autres robots puissent nous détecter
void ping_robots(void) {
    uint8_t ping_message[7] = {"robots"};
    pogobot_infrared_sendLongMessage_omniSpe(ping_message, sizeof(ping_message));
}

// Met à jour le tableau `detection[]` avec les directions d'où des messages ont été reçus
void observe(bool detection[]) {
    pogobot_infrared_update();
    while (pogobot_infrared_message_available()) {
        message_t msg;
        pogobot_infrared_recover_next_message(&msg);
        int sensor_id = msg.header._receiver_ir_index;
        if (sensor_id >= 0 && sensor_id < 5) {
            detection[sensor_id] = true;
        }
    }
}

// Fonction de déplacement vers l'avant
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

// Tourne à gauche
void move_left(void) {
    mydata->lastTurn = LEFT_TURN;
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

// Tourne à droite
void move_right(void) {
    mydata->lastTurn = RIGHT_TURN;
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
        pogobot_motor_dir_set(motorR, (mydata->dirRight + 1) % 2);
    } else {
        pogobot_motor_set(motorL, motorHalf);
        pogobot_motor_set(motorR, motorStop);
    }
#endif
}

// Stoppe les deux moteurs
void move_stop(void) {
    pogobot_motor_set(motorL, motorStop);
    pogobot_motor_set(motorR, motorStop);
}

// Applique la logique de dispersion selon la direction d'où viennent les autres robots
void move_logic(bool direction_detected[]) {
    int r = 0, g = 0, b = 0;

    if (direction_detected[0]) {
        move_left(); r = 255; g = 128; b = 0; // robot détecté devant
    } else if (direction_detected[1] && !direction_detected[3]) {
        move_left(); r = 255; g = 255; b = 0; // robot à droite uniquement
    } else if (direction_detected[3] && !direction_detected[1]) {
        move_right(); r = 255; g = 255; b = 0; // robot à gauche uniquement
    } else if (direction_detected[2]) {
        move_front(); r = 0; g = 255; b = 0; // robot derrière
    } else if (direction_detected[1] && direction_detected[3]) {
        move_front(); r = 0; g = 255; b = 255; // robots à gauche ET à droite
    } else {
        move_stop(); r = 0; g = 0; b = 255; // aucun robot détecté
    }

    pogobot_led_setColor(r, g, b); // feedback visuel via LED
}

// Initialisation de base du robot (lecture calibration moteurs, seed aléatoire, etc.)
void user_init(void) {
    pogobot_stopwatch_reset(&mydata->timer_it);
    main_loop_hz = 45;
    max_nb_processed_msg_per_tick = 0;
    msg_rx_fn = NULL;
    msg_tx_fn = NULL;
    error_codes_led_idx = 3;
    pogobot_infrared_set_power(3);
    srand(pogobot_helper_getRandSeed());

    uint16_t power_mem[3];
    uint8_t dir_mem[3];
    pogobot_motor_power_mem_get(power_mem);
    pogobot_motor_dir_mem_get(dir_mem);

    mydata->motorLeft = power_mem[1];
    mydata->motorRight = power_mem[0];
    mydata->dirLeft = dir_mem[1];
    mydata->dirRight = dir_mem[0];
    mydata->lastTurn = NO_TURN;
}

// Fonction appelée à chaque cycle du robot (loop principale)
void user_step(void) {
    bool direction_detected[4] = {false, false, false, false}; // capteurs IR dans chaque direction
    ping_robots();                         // Envoie sa présence
    observe(direction_detected);           // Analyse les signaux reçus
    move_logic(direction_detected);        // Applique un comportement en fonction
}

// Envoie un message spécial pour représenter un mur (dans la simulation)
bool ping_walls(void) {
    uint8_t ping_message[5] = {"wall"};
    return pogobot_infrared_sendLongMessage_omniSpe(ping_message, sizeof(ping_message));
}

// Initialisation spécifique aux murs (dans la simulation uniquement)
void walls_user_init(void) {
#ifndef SIMULATOR
    printf("setup ok\n");
#endif
    srand(pogobot_helper_getRandSeed());
    pogobot_infrared_set_power(3);
    main_loop_hz = 45;
    max_nb_processed_msg_per_tick = 0;
    percent_msgs_sent_per_ticks = 100;
    msg_rx_fn = NULL;
    msg_tx_fn = ping_walls;
    error_codes_led_idx = -1;
}

// Comportement simulé des murs (rien ici mais prêt à étendre)
void walls_user_step(void) {
    bool direction_detected[4] = {false, false, false, false};
    observe(direction_detected);
    move_logic(direction_detected);
}

// Point d'entrée principal
int main(void) {
    pogobot_init();
    pogobot_start(user_init, user_step);                     // Comportement du robot
    pogobot_start(walls_user_init, walls_user_step);         // Comportement des murs
    return 0;
}
