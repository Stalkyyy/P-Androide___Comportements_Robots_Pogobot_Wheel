#include "pogobase.h"
#include <stdlib.h>
#include <time.h>

#define INFRARED_POWER 3

#define MAX_ROBOTS 10
#define NB_DIRECTIONS 5

#define HAS_WHEEL false // Permet de choisir le cas où c'est un robot à roue, ou un robot à brosse.

#define NO_TURN 'N'
#define LEFT_TURN 'L'
#define RIGHT_TURN 'R'

/*
 * ====================================================================================
 */

// Prototypes des fonctions
void move_front(void);
void move_left(void);
void move_right(void);
void move_stop(void);

void random_or_follow_turn(void);
void move_logic(bool *detection);

void ping_robots(void);
void observe(uint8_t cpt_dir[NB_DIRECTIONS], uint16_t id_robots_dir[NB_DIRECTIONS][MAX_ROBOTS], bool detection_mur[NB_DIRECTIONS]);
uint8_t robot_vu(uint8_t cpt[NB_DIRECTIONS]);
uint8_t dir_max(uint8_t cpt[NB_DIRECTIONS]);
uint8_t to_follow(uint8_t cpt_dir[NB_DIRECTIONS], uint16_t id_robots_dir[NB_DIRECTIONS][MAX_ROBOTS], uint8_t nb_robots_max, uint16_t *id_comparatif);
void move_follow(uint8_t direction_suivie);

bool ping_walls(void);
void walls_user_init(void);
void walls_user_step(void);

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


void move_logic(bool *detection) {
    // Initialisation des capteurs
    bool sensorFront = detection[0];
    bool sensorRight = detection[1];
    // bool sensorBack = detection[2];
    bool sensorLeft = detection[3];
    bool sensorEverywhere = detection[4]; // Dans le cas du simulateur.

    // Pour les leds
    int r = 0, g = 0, b = 0;

    if ((sensorFront && sensorLeft && sensorRight) || sensorEverywhere) {
    #ifdef SIMULATOR
        if (pogobot_ticks % 4 == 0)
            move_front();
        else
            random_or_follow_turn();
    #else
        if (HAS_WHEEL)
            random_or_follow_turn();
        else
            move_stop();
    #endif
    }

    else if (sensorLeft && sensorRight) {
        move_front();
    } 

    else if (sensorRight) {
        move_left();
    }

    else if (sensorLeft) {
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


void ping_robots(void) {
    uint8_t ping_message[7] = {"robots"};
    pogobot_infrared_sendLongMessage_omniSpe(ping_message, sizeof(ping_message));
}

void observe(uint8_t cpt_dir[NB_DIRECTIONS], uint16_t id_robots_dir[NB_DIRECTIONS][MAX_ROBOTS], bool detection_mur[5]){
    pogobot_infrared_update();
    while(pogobot_infrared_message_available()){
        message_t msg;
        pogobot_infrared_recover_next_message(&msg);

        // si on détecte un robot
        if(memcmp("robots", msg.payload, sizeof("robots")) == 0){
            uint8_t idx = msg.header._receiver_ir_index;

            // on stocke les ids des robots selon les directions en évitant les doublons d'ids
            uint8_t dejaDedans = 0;
            for(int i=0; i<cpt_dir[idx]; i++){
                if(id_robots_dir[idx][i] == msg.header._sender_id){
                    dejaDedans = 1;
                    break;
                }
            }
            
            if(!dejaDedans){
                // on stocke l'id du robot qui a envoyé la direction
                id_robots_dir[idx][cpt_dir[idx]] = msg.header._sender_id;

                // on stocke le nombre de robots ayant envoyé cette direction
                cpt_dir[idx] ++;
            }
        } 
        // si on détecte un mur, on l'évite si on ne suit personne
        else {
            int sensor_id = msg.header._receiver_ir_index;

            if (sensor_id >= 0 && sensor_id < 5) {
                detection_mur[sensor_id] = true;
            }
        }
    }
}

uint8_t robot_vu(uint8_t cpt[NB_DIRECTIONS]) {
    for(int i=0; i<NB_DIRECTIONS; i++){
        if(cpt[i] > 0){
            return 1;
        }
    }
    return 0;
}

uint8_t dir_max(uint8_t cpt[NB_DIRECTIONS]) {
    uint8_t nb_robots = 0;
    for(int i=0; i<NB_DIRECTIONS; i++){
        if(nb_robots < cpt[i]){
            nb_robots = cpt[i];
        }
    }
    return nb_robots;
}

uint8_t to_follow(uint8_t cpt_dir[NB_DIRECTIONS], uint16_t id_robots_dir[NB_DIRECTIONS][MAX_ROBOTS], uint8_t nb_robots_max, uint16_t *id_comparatif){
    uint8_t direction_suivie = 0;
    uint16_t robot_suivi = UINT16_MAX;
    uint16_t mon_id = pogobot_helper_getid();

    for(int i=0; i<NB_DIRECTIONS; i++){
        // parmi toutes les directions, si c'est l'une de celles avec le plus de membres détectés
        if(cpt_dir[i] == nb_robots_max){

            // on parcourt tous les ids des robots
            for(int j=0; j<nb_robots_max; j++){
                uint16_t id_voisin = id_robots_dir[i][j];

                // et s'il est plus petit que mon id et le plus proche de mon id, alors je garde en mémoire le robot à suivre et la direction à prendre 
                if(mon_id > id_voisin && (mon_id-id_voisin) <= *id_comparatif){

                    // si c'est le même robot que le meilleur mais que cette fois on le détecte à droite ou à gauche, on privilégie le fait de tourner 
                    if(robot_suivi == id_voisin && (i==1 || i==3)){
                        direction_suivie = i;
                    }

                    // si c'est bien un meilleur robot, alors on le garde en mémoire ainsi que sa direction
                    if(robot_suivi != id_voisin){
                        *id_comparatif = mon_id-id_voisin;
                        direction_suivie = i;
                        robot_suivi = id_voisin;
                    }
                }
            }
        }
    }
    return direction_suivie;
}

void move_follow(uint8_t direction_suivie){
    pogobot_led_setColor(255, 0, 0);
    switch(direction_suivie) {
        case 0 :
            move_front();
            break;
        case 1 :
            move_right();
            break;
        default :
            move_left();
            break;
    }   
}


/*
 * ====================================================================================
 */


// Init function. Called once at the beginning of the program (cf 'pogobot_start' call in main())
void user_init(void) {
#ifndef SIMULATOR
    printf("setup ok\n");
#endif

    // Init timer
    pogobot_stopwatch_reset(&mydata->timer_it);

    // Set main loop frequency, message sending frequency, message processing frequency
    main_loop_hz = 45;      // Call the 'user_step' function 60 times per second
    max_nb_processed_msg_per_tick = 0;
    // Specify functions to send/transmit messages. See the "hanabi" example to see message sending/processing in action!
    msg_rx_fn = NULL;       // If Null, no reception of message
    msg_tx_fn = NULL;       // If Null, don't send any message

    // Set led index to show error codes (e.g. time overflows)
    error_codes_led_idx = 3; // Default value, negative values to disable

    pogobot_infrared_set_power(INFRARED_POWER);
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


// Step function. Called continuously at each step of the pogobot main loop
void user_step(void) {

    // Transmission d'un message
    ping_robots();

    uint8_t cpt_dir[NB_DIRECTIONS] = {0}; // compteur des senseurs des directions reçues
    uint16_t id_robots_dir[NB_DIRECTIONS][MAX_ROBOTS]; // stockage des ids des robots détectés en fct des directions
    for (int i = 0; i < NB_DIRECTIONS; i++) {
        for (int j = 0; j < MAX_ROBOTS; j++) {
            id_robots_dir[i][j] = UINT16_MAX;
        }
    }

    bool detection_mur[5] = {false, false, false, false, false};
    
    // Réception d'un message
    observe(cpt_dir, id_robots_dir, detection_mur);
    
    // on regarde si le robot a vu des voisins qu'il peut suivre
    uint8_t vu = robot_vu(cpt_dir);

    // s'il a trouvé qlq à suivre, il s'adapte à la direction majoritaire
    if(vu){

        // on cherche le nombre de robots le plus important parmi toutes les directions
        uint8_t nb_robots_max = dir_max(cpt_dir);

        // on récupère l'id du robot suivi et sa direction
        uint16_t id_comparatif = UINT16_MAX;
        uint8_t direction_suivie = to_follow(cpt_dir, id_robots_dir, nb_robots_max, &id_comparatif);

        // Mouvement
        // si je n'ai trouvé personne à suivre, j'avance aléatoirement en évitant les murs si nécessaire
        if(id_comparatif == UINT16_MAX){
            move_logic(detection_mur);
        } else { // sinon je tourne dans sa direction et je le suis
            move_follow(direction_suivie);
        }

    } else { // s'il n'a vu personne d'intérêt et aucun mur, il continue d'avancer aléatoirement
        move_logic(detection_mur);
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
