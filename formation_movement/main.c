// Main include for pogobots, both for real robots and for simulations
#include "pogobase.h"

#define INFRARED_POWER 3 // 1, 2, 3

#define MAX_ROBOTS 10
#define NB_DIRECTIONS 4

#define HAS_WHEEL false // Permet de choisir le cas où c'est un robot à roue, ou un robot à brosse.


/*
 * ====================================================================================
 */

// Prototypes des fonctions
void move_front(void);
void move_left(void);
void move_right(void);
void move_stop(void);
uint8_t robot_vu(uint8_t cpt[NB_DIRECTIONS]);
uint8_t dir_max(uint8_t cpt[NB_DIRECTIONS]);

/*
 * ====================================================================================
 */


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

    uint16_t id_robot_suivi;

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

        pogobot_motor_dir_set(motorL, (mydata->dirLeft + 1) % 2);
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
        pogobot_motor_dir_set(motorR, (mydata->dirRight + 1) % 2);
    } else {
        pogobot_motor_set(motorL, motorHalf);
        pogobot_motor_set(motorR, motorStop);
    }
}

void move_stop(void) {
    pogobot_motor_set(motorL, motorStop);
    pogobot_motor_set(motorR, motorStop);
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

    mydata->id_robot_suivi = UINT16_MAX;
}


// Step function. Called continuously at each step of the pogobot main loop
void user_step(void) {

    // Transmission d'un message
    uint8_t msg_robot[] = "robot";    
    pogobot_infrared_sendLongMessage_omniSpe(msg_robot, sizeof(msg_robot));

    // Réception d'un message et décision du mouvement à faire
    pogobot_infrared_update();

    uint8_t cpt_dir[NB_DIRECTIONS] = {0}; // compteur des senseurs des directions reçues
    uint16_t id_robots_dir[NB_DIRECTIONS][MAX_ROBOTS]; // stockage des ids des robots détectés en fct des directions
    for (int i = 0; i < NB_DIRECTIONS; i++) {
        for (int j = 0; j < MAX_ROBOTS; j++) {
            id_robots_dir[i][j] = UINT16_MAX;
        }
    }
    
    while(pogobot_infrared_message_available()>=1){
        message_t msg;
        pogobot_infrared_recover_next_message(&msg);

        // si on détecte un robot
        if(memcmp(msg_robot, msg.payload, sizeof(msg_robot)) == 0){
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
        // si on détecte autre / un mur (on verra plus tard)
        else {
            printf("Mur !\n");
            move_right(); // on évite le mur 
        }
    }
    
    // on regarde si le robot a vu des voisins qu'il peut suivre
    uint8_t vu = robot_vu(cpt_dir);

    // s'il a trouvé qlq à suivre, il s'adapte à la direction majoritaire
    if(vu){

        // on cherche le nombre de robots le plus important parmi toutes les directions
        uint8_t nb_robots_max = dir_max(cpt_dir);

        // on récupère l'id du robot suivi et sa direction
        uint8_t direction_suivie = 0;
        uint16_t robot_suivi = UINT16_MAX;
        uint16_t id_comparatif = UINT16_MAX;
        uint16_t mon_id = pogobot_helper_getid();

        for(int i=0; i<NB_DIRECTIONS; i++){
            // parmi toutes les directions, si c'est l'une de celles avec le plus de membres détectés
            if(cpt_dir[i] == nb_robots_max){

                // on parcourt tous les ids des robots
                for(int j=0; j<nb_robots_max; j++){
                    uint16_t id_voisin = id_robots_dir[i][j];

                    // et s'il est plus petit que mon id et le plus proche de mon id, alors je garde en mémoire le robot à suivre et la direction à prendre 
                    if(mon_id > id_voisin && (mon_id-id_voisin) <= id_comparatif){

                        // si c'est le même robot que le meilleur mais que cette fois on le détecte à droite ou à gauche, on privilégie le fait de tourner 
                        if(robot_suivi == id_voisin && (i==1 || i==3)){
                            direction_suivie = i;
                        }

                        // si c'est bien un meilleur robot, alors on le garde en mémoire ainsi que sa direction
                        if(robot_suivi != id_voisin){
                            id_comparatif = mon_id-id_voisin;
                            direction_suivie = i;
                            robot_suivi = id_voisin;
                        }
                    }
                }
            }
        }

        // Mouvement
        // si je n'ai trouvé personne à suivre, j'avance tout droit
        if(id_comparatif == UINT16_MAX){
            pogobot_led_setColor(0, 255, 0); 
            move_front();
        } else { // sinon je tourne dans sa direction et je le suis
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

            // jsp encore à quoi cv servir mais on verra bien
            mydata->id_robot_suivi = robot_suivi;
        }

    } else { // s'il n'a vu personne d'intérêt, il continue sa vie
        pogobot_led_setColor(0, 255, 0); 
        move_front();
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