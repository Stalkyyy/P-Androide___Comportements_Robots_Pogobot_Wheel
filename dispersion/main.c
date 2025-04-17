#include "pogobase.h"
#include <stdlib.h>
#include <math.h>

// Définitions de constantes pour les calculs d'angle et les comportements
#ifndef M_PI_4
#define M_PI_4 0.7853981633974483f
#endif

#define HAS_WHEEL true // Indique si le robot utilise des roues
#define NO_TURN 'N'
#define LEFT_TURN 'L'
#define RIGHT_TURN 'R'

// Délais et distances pour les comportements
#define DETECTION_TIMEOUT_MS 4000 // Temps avant de considérer qu'il n'y a plus de robots détectés
#define DISPERSION_INERTIE_TICKS 4000 // Durée d'inertie pour maintenir une direction
#define BLOCKED_TIMEOUT_MS 3000 // Temps avant de considérer que le robot est bloqué

#define DISTANCE_CLOSE 50    // Distance proche (en unités arbitraires)
#define DISTANCE_MEDIUM 150  // Distance moyenne (en unités arbitraires)
#define DISTANCE_FAR 300     // Distance lointaine (en unités arbitraires)

// Structure pour stocker les données utilisateur
typedef struct {
    uint8_t data_foo[8]; // Données utilisateur génériques
    time_reference_t timer_it; // Chronomètre pour mesurer le temps

    uint16_t motorLeft;  // Puissance du moteur gauche
    uint8_t dirLeft;     // Direction du moteur gauche

    uint16_t motorRight; // Puissance du moteur droit
    uint8_t dirRight;    // Direction du moteur droit

    char lastTurn;       // Dernière direction prise (gauche ou droite)

    int direction_counts[4]; // Compteur de robots détectés dans chaque direction
    int direction_distances[4]; // Distance estimée pour chaque direction
    uint32_t last_detect;    // Dernière détection d'un robot
    uint32_t last_movement;  // Dernier mouvement significatif
    int inertie_tick;        // Ticks d'inertie pour maintenir une direction

    int last_position;       // Dernière position significative
    int random_movement_ticks; // Ticks pour maintenir un mouvement aléatoire
} USERDATA;

DECLARE_USERDATA(USERDATA);
REGISTER_USERDATA(USERDATA);

// Fonction pour avancer tout droit
void move_front(void) {
    pogobot_motor_set(motorL, mydata->motorLeft);
    pogobot_motor_set(motorR, mydata->motorRight);
    pogobot_led_setColor(0, 255, 0); // LED verte : avancer normalement
}

// Fonction pour tourner à gauche
void move_left(void) {
    mydata->lastTurn = LEFT_TURN;
    pogobot_motor_set(motorL, motorStop);
    pogobot_motor_set(motorR, mydata->motorRight);
    pogobot_led_setColor(255, 255, 0); // LED jaune : tourner à gauche
}

// Fonction pour tourner à droite
void move_right(void) {
    mydata->lastTurn = RIGHT_TURN;
    pogobot_motor_set(motorL, mydata->motorLeft);
    pogobot_motor_set(motorR, motorStop);
    pogobot_led_setColor(255, 255, 0); // LED jaune : tourner à droite
}

// Fonction pour s'arrêter
void move_stop(void) {
    pogobot_motor_set(motorL, motorStop);
    pogobot_motor_set(motorR, motorStop);
    pogobot_led_setColor(255, 0, 0); // LED rouge : arrêt
}

// Fonction pour mettre à jour les directions et les distances
void update_direction_count(void) {
    // Réinitialise les compteurs et les distances
    for (int i = 0; i < 4; i++) {
        mydata->direction_counts[i] = 0;
        mydata->direction_distances[i] = DISTANCE_FAR; // Par défaut, aucune détection
    }

    // Parcourt les messages infrarouges reçus
    pogobot_infrared_update();
    while (pogobot_infrared_message_available()) {
        message_t mess;
        pogobot_infrared_recover_next_message(&mess);

        int dir = mess.header._receiver_ir_index; // Direction du capteur ayant reçu le message
        int distance = rand() % (DISTANCE_FAR + 1); // Simule une distance entre 0 et DISTANCE_FAR

        if (dir >= 0 && dir < 4) {
            mydata->direction_counts[dir]++;
            mydata->direction_distances[dir] = distance; // Stocke la distance simulée
        }
    }
}

// Fonction principale pour gérer la dispersion
void perform_dispersion_movement(void) {
    int *dir_count = mydata->direction_counts;
    int *dir_distances = mydata->direction_distances;

    // Vérifie la densité locale (nombre total de robots détectés)
    int total_presence = 0;
    for (int i = 0; i < 4; i++) {
        total_presence += dir_count[i];
    }

    if (total_presence >= 5) {
        // Fuite d'urgence : éviter de rester coincé dans un groupe
        int choice = rand() % 2;
        if (choice == 0) {
            pogobot_motor_set(motorL, motorThreeQuarter);
            pogobot_motor_set(motorR, motorQuarter);
        } else {
            pogobot_motor_set(motorL, motorQuarter);
            pogobot_motor_set(motorR, motorThreeQuarter);
        }
        pogobot_led_setColor(255, 0, 255);  // LED magenta : fuite densité
        return;
    }

    // Calcul des forces de dispersion en fonction des distances
    int vx = 0, vy = 0;
    for (int i = 0; i < 4; i++) {
        int f = 0;
        if (dir_distances[i] < DISTANCE_CLOSE) f = 30;
        else if (dir_distances[i] < DISTANCE_MEDIUM) f = 20;
        else if (dir_distances[i] < DISTANCE_FAR) f = 10;

        switch (i) {
            case 0: vy += f; break; // Avant
            case 1: vx += f; break; // Droite
            case 2: vy -= f; break; // Arrière
            case 3: vx -= f; break; // Gauche
        }
    }

    // Si aucune force n'est appliquée, effectuer un mouvement par défaut
    if (vx == 0 && vy == 0) {
        if (mydata->random_movement_ticks == 0) {
            mydata->random_movement_ticks = 60 + rand() % 100; // Durée aléatoire pour avancer

            int choice = rand() % 3;
            if (choice == 0) {
                pogobot_motor_set(motorL, motorHalf);
                pogobot_motor_set(motorR, motorHalf);
            } else if (choice == 1) {
                pogobot_motor_set(motorL, motorQuarter);
                pogobot_motor_set(motorR, motorThreeQuarter);
            } else {
                pogobot_motor_set(motorL, motorThreeQuarter);
                pogobot_motor_set(motorR, motorQuarter);
            }
        } else {
            mydata->random_movement_ticks--;
        }

        pogobot_led_setColor(0, 0, 255); // LED bleue : mouvement par défaut
        return;
    }

    // Applique les forces inversées pour se disperser
    vx = -vx;
    vy = -vy;
    float angle = atan2f((float)vy, (float)vx);

    // Détermine la direction en fonction de l'angle
    if (angle > -M_PI_4 && angle <= M_PI_4) {
        move_right();
    } else if (angle > M_PI_4 && angle <= 3 * M_PI_4) {
        move_front();
    } else if (angle < -M_PI_4 && angle >= -3 * M_PI_4) {
        move_stop();
    } else {
        move_left();
    }
    pogobot_led_setColor(255, 128, 0); // LED orange : dispersion
}

// Fonction pour envoyer un ping aux autres robots
void ping_robots(void) {
    uint8_t ping_message[7] = {"robots"};
    pogobot_infrared_sendLongMessage_omniSpe(ping_message, sizeof(ping_message));
}

// Fonction d'initialisation
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
    mydata->motorLeft = power_mem[1];
    mydata->motorRight = power_mem[0];

    pogobot_motor_dir_mem_get(dir_mem);
    mydata->dirLeft = dir_mem[1];
    mydata->dirRight = dir_mem[0];

    mydata->lastTurn = NO_TURN;
    mydata->inertie_tick = 0;
    mydata->last_detect = current_time_milliseconds();
    mydata->last_movement = current_time_milliseconds();
    mydata->last_position = 0;
    mydata->random_movement_ticks = DISPERSION_INERTIE_TICKS / 2;
}

// Fonction appelée à chaque étape
void user_step(void) {
    ping_robots(); // Envoie un ping pour détecter les autres robots
    update_direction_count(); // Met à jour les directions et les distances
    perform_dispersion_movement(); // Applique la logique de dispersion
}

// Point d'entrée du programme
int main(void) {
    pogobot_init(); // Initialise le robot
    pogobot_start(user_init, user_step); // Démarre la boucle principale
    return 0;
}