#include "pogobase.h"
#include <stdlib.h>
#include <time.h>

#define HAS_WHEEL false// Active les réglages spécifiques pour robots à roues
#define motorSpeed 1023 // Puissance maximale moteur

#define NO_TURN 'N'
#define LEFT_TURN 'L'
#define RIGHT_TURN 'R'

// Seuil à partir duquel le robot réagit à la présence d'autres robots
#define SEUIL_DETECTION_DISPERSION 2

// Structure contenant les paramètres du robot
typedef struct {
    time_reference_t timer_it; // Timer interne pour les mesures temporelles
    uint16_t motorLeft;        // Puissance du moteur gauche
    uint8_t dirLeft;           // Direction du moteur gauche
    uint16_t motorRight;       // Puissance du moteur droit
    uint8_t dirRight;          // Direction du moteur droit
    char lastTurn;             // Dernier sens de rotation utilisé
} USERDATA;

DECLARE_USERDATA(USERDATA);
REGISTER_USERDATA(USERDATA);

// Fonction d'envoi d'un message IR de présence
void ping_robots(void) {
    uint8_t ping_message[7] = {"robots"};
    pogobot_infrared_sendLongMessage_omniSpe(ping_message, sizeof(ping_message));
}

// Mouvement vers l'avant
void move_front(void) {
    if (HAS_WHEEL) {
        pogobot_motor_set(motorL, mydata->motorLeft);
        pogobot_motor_set(motorR, mydata->motorRight);
        pogobot_motor_dir_set(motorL, mydata->dirLeft);
        pogobot_motor_dir_set(motorR, mydata->dirRight);
    } else {
        pogobot_motor_set(motorL, motorSpeed);
        pogobot_motor_set(motorR, motorSpeed);
    }
}

// Rotation vers la gauche
void move_left(void) {
    mydata->lastTurn = LEFT_TURN;
    if (HAS_WHEEL) {
        pogobot_motor_set(motorL, motorHalf);
        pogobot_motor_set(motorR, motorHalf);
        pogobot_motor_dir_set(motorL, (mydata->dirLeft + 1) % 2); // inversion direction gauche
        pogobot_motor_dir_set(motorR, mydata->dirRight);
    } else {
        pogobot_motor_set(motorL, motorStop);
        pogobot_motor_set(motorR, motorHalf);
    }
}

// Rotation vers la droite
void move_right(void) {
    mydata->lastTurn = RIGHT_TURN;
    if (HAS_WHEEL) {
        pogobot_motor_set(motorL, motorHalf);
        pogobot_motor_set(motorR, motorHalf);
        pogobot_motor_dir_set(motorL, mydata->dirLeft);
        pogobot_motor_dir_set(motorR, (mydata->dirRight + 1) % 2); // inversion direction droite
    } else {
        pogobot_motor_set(motorL, motorHalf);
        pogobot_motor_set(motorR, motorStop);
    }
}

// Arrêt complet
void move_stop(void) {
    pogobot_motor_set(motorL, motorStop);
    pogobot_motor_set(motorR, motorStop);
}

// Initialisation utilisateur : lecture des paramètres moteur
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

// Fonction appelée à chaque tick
// Si le nombre de robots détectés dépasse le seuil, le robot entre en mode dispersion
void user_step(void) {
    // Tableau booléen pour marquer les directions où un robot est détecté
    bool direction_detected[4] = {false, false, false, false}; // 0: avant, 1: droite, 2: arrière, 3: gauche
    ping_robots();
    pogobot_infrared_update();

    while (pogobot_infrared_message_available()) {
        message_t msg;
        pogobot_infrared_recover_next_message(&msg);
        int ir_direction = msg.header._receiver_ir_index;
        if (ir_direction >= 0 && ir_direction < 4) {
            direction_detected[ir_direction] = true;
        }
    }

    // Compte combien de directions ont détecté un robot
    int total_detected = 0;
    for (int i = 0; i < 4; i++) {
        if (direction_detected[i]) total_detected++;
    }

    // Si assez de robots sont détectés, entre en mode dispersion
    if (total_detected >= SEUIL_DETECTION_DISPERSION) {
        move_left(); // stratégie simple : tourner à gauche
        pogobot_led_setColor(255, 0, 255); // LED magenta pour alerte
        return;
    }



    // Couleurs RGB pour feedback LED
    int r = 0, g = 0, b = 0;

    // Logique de déplacement selon les directions détectées
    if (direction_detected[1] && !direction_detected[3]) {
        move_left(); r = 255; g = 255; b = 0; // robot à droite
    } else if (direction_detected[3] && !direction_detected[1]) {
        move_right(); r = 255; g = 255; b = 0; // robot à gauche
    } else if (direction_detected[2]) {
        move_front(); r = 0; g = 255; b = 0; // robot derrière
    } else if (direction_detected[1] && direction_detected[3]) {
        move_front(); r = 0; g = 255; b = 255; // robot des deux côtés
    } else {
        move_stop(); r = 0; g = 0; b = 255; // aucun robot
    }
    pogobot_led_setColor(r,g,b); // mise à jour de la couleur LED

   

 
}

int main(void) {
    pogobot_init();
    pogobot_start(user_init, user_step);
    return 0;
}
