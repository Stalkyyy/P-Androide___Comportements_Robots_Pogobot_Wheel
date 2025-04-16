// Main include for pogobots, both for real robots and for simulations
#include "pogobase.h"

#define INFRARED_POWER 3 // 1, 2, 3

#define MAX_ROBOTS 10

uint8_t sens_robot[4][4] = {
//   F  R  B  L
    {1, 1, 0, 3},
    {3, 1, 1, 0},
    {0, 3, 1, 1},
    {1, 0, 3, 1},
};

// "Global" variables should be inserted within the USERDATA struct.
// /!\  In simulation, don't declare non-const global variables outside this struct, elsewise they will be shared among all agents (and this is not realistic).
typedef struct {
    // Put all global variables you want here.
    uint8_t data_foo[8];
    time_reference_t timer_it;
    uint8_t last_move; // le mouvement précédemment effectué 
    uint8_t id_robots_suivis[MAX_ROBOTS]; // la liste des ids des robots qu'on suit
    uint8_t nb_robots_suivis; // le nombre des voisins qu'on suit
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

    mydata->last_move = 5;
    //mydata->id_robots_suivis = {0}; // id du robot qu'on suit 
    mydata->nb_robots_suivis = 0;
}


// Step function. Called continuously at each step of the pogobot main loop
void user_step(void) {

    // Transmission d'un message
    uint8_t msg_envoi[MAX_ROBOTS+2];
    msg_envoi[0] = mydata->last_move;
    msg_envoi[1] = mydata->nb_robots_suivis;
    for(int i=0; i<MAX_ROBOTS; i++){
        msg_envoi[i+2] = mydata->id_robots_suivis[i];
    }
    
    pogobot_infrared_sendLongMessage_omniSpe(msg_envoi, sizeof(msg_envoi));

    // Réception d'un message et décision du mouvement à faire
    uint8_t move_id = 5;
    pogobot_infrared_update();

    uint8_t cpt_dir[4] = {0}; // compteur des senseurs des directions reçues
    uint8_t id_robots_dir[4][MAX_ROBOTS] = {0}; // stockage des ids des robots détectés en fct des directions

    uint8_t last_moves[4] = {0}; // on garde le dernier mouvement effectué par les formations
    uint8_t senseurs_detection[4] = {0}; // on garde les senseurs d'où les messages ont été envoyés pour détecter si le robot et la formation sont dans le même sens
    
    while(pogobot_infrared_message_available()>=1){
        message_t msg;
        pogobot_infrared_recover_next_message(&msg);

        // si on détecte un robot
        if(msg.header._packet_type == ir_t_user){
            uint8_t skip = 0;

            // si c'est l'un des robots qui nous suit déjà alors on l'ignore
            for(int i=0; i<msg.payload[1]; i++){
                if(msg.payload[i+2] == pogobot_helper_getid()){
                    skip = 1;
                    break;
                }
            }
            if(skip==1){
                continue;
            }

            // on stocke l'id du robot qui a envoyé la direction
            id_robots_dir[msg.header._receiver_ir_index][cpt_dir[msg.header._receiver_ir_index]] = msg.header._sender_id;
            
            // on stocke le nombre de robots ayant envoyé cette direction
            cpt_dir[msg.header._receiver_ir_index] ++;

            // on stocke le dernier mouvement dans le champ adapté
            last_moves[msg.header._receiver_ir_index] = msg.payload[0];

            // on stocke par quel senseur le message a été envoyé
            senseurs_detection[msg.header._receiver_ir_index] =  msg.header._sender_ir_index;
        } 
        // si on détecte autre / un mur (à implémenter selon ce que va donner wall_allignment)
        else {
            printf("Mur !\n");
            move_id = 1; // on évite le mur 
        }
    }

    
    // on regarde si le robot a vu des voisins qu'il peut suivre
    uint8_t pas_vu = 0;
    for(int i=0; i<4; i++){
        if(cpt_dir[i] > 0){
            pas_vu = 1;
        }
    }

    // s'il a trouvé qlq à suivre, il s'adapte à la direction majoritaire
    if(pas_vu == 1){

        // on cherche la direction où y a le plus de robots
        uint8_t nb_robots_max = 0;
        for(int i=0; i<4; i++){
            if(nb_robots_max < cpt_dir[i]){
                nb_robots_max = cpt_dir[i];
            }
        }

        // on vérifie si y a une égalité entre plusieurs directions
        uint8_t dir_egal[4] = {0};
        uint8_t cpt_egalite = 0;
        for(int i=0; i<4; i++){
            if(nb_robots_max == cpt_dir[i]){
                dir_egal[cpt_egalite] = i;
                cpt_egalite++;
            }
        }

        // si c'est 1 vs 1 : celui avec le plus petit id suit l'autre
        if(nb_robots_max == 1){
            uint8_t max_id = pogobot_helper_getid();
            uint8_t max_dir = 0;
            for(int i=0; i<cpt_egalite; i++){
                uint8_t id_voisin = id_robots_dir[dir_egal[i]][0];
                if(max_id < id_voisin){
                    max_id = id_voisin;
                    max_dir = dir_egal[i];
                }
            }
            if(max_id == pogobot_helper_getid()){
                pogobot_led_setColor(0, 255, 0); 
            } else {

                // si mvts inverses 
                if(senseurs_detection[max_dir] == max_dir){ // si le message a été envoyé et reçu du même côté alors sens inverse donc on tourne
                    move_id = 1;
                } else { // si dans le même sens, il recopie le mouvement
                    move_id = last_moves[max_dir];
                }

                // avec le tableau de sens
                /*move_id = sens_robot[max_dir][senseurs_detection[max_dir]];
                if(move_id == 0){
                    move_id = last_moves[max_dir]; 
                }*/

                // on récupère les ids des voisins qu'on suit 
                memcpy(mydata->id_robots_suivis, id_robots_dir[max_dir], MAX_ROBOTS * sizeof(uint8_t));
                mydata->nb_robots_suivis = nb_robots_max;
                pogobot_led_setColor(255, 0, 0);
            }
    
        } else { // s'il détecte un ou plusieurs groupes
            uint8_t idx_robots_dir = 0;
            // si y a égalité, on prend l'une des directions exécutées au hasard
            if(cpt_egalite > 1){
                uint8_t idx = rand() % cpt_egalite;

                // si mvts inverses
                if(senseurs_detection[dir_egal[idx]] == dir_egal[idx]){ // si le message a été envoyé et reçu du même côté alors sens inverse donc on tourne
                    move_id = 1;
                } else { // si dans le même sens, il recopie le mouvement
                    move_id = last_moves[dir_egal[idx]];
                }

                // avec le tableau de sens
                /*move_id = sens_robot[dir_egal[idx]][senseurs_detection[dir_egal[idx]]];
                if(move_id == 0){
                    move_id = last_moves[dir_egal[idx]]; 
                }*/

                idx_robots_dir = dir_egal[idx];
            } else {

                // si mvts inverses
                if(senseurs_detection[dir_egal[0]] == dir_egal[0]){ // si le message a été envoyé et reçu du même côté alors sens inverse donc on tourne
                    move_id = 1;
                } else {
                    move_id = last_moves[dir_egal[0]];
                }

                // avec le tableau de sens
                /*move_id = sens_robot[dir_egal[0]][senseurs_detection[dir_egal[0]]];
                if(move_id == 0){
                    move_id = last_moves[dir_egal[0]]; 
                }*/

                idx_robots_dir = dir_egal[0];
            }

            // on récupère les ids des voisins qu'on suit 
            memcpy(mydata->id_robots_suivis, id_robots_dir[idx_robots_dir], MAX_ROBOTS * sizeof(uint8_t));
            mydata->nb_robots_suivis = nb_robots_max;
            pogobot_led_setColor(255, 0, 0);
        }

    } else { // s'il n'a vu personne d'intérêt, il continue sa vie
        pogobot_led_setColor(0, 255, 0); 
    }

    // on garde en mémoire le dernier mouvement effectué pour le transmettre aux membres de la formation suivants
    mydata->last_move = move_id;

    // Mouvement 

    // s'il le détecte à droite alors il va à droite ou s'il le détecte en arrière il va à droite
    // et s'il le détecte en face il tourne un peu à droite pour éviter de le foncer dedans et ensuite le suivre
    if (move_id == 1 || move_id == 2) { 
        pogobot_motor_set(motorL, motorHalf);
        pogobot_motor_set(motorR, motorStop);
    } else if (move_id == 3) { // s'il le détecte à gauche alors il tourne à gauche
        pogobot_motor_set(motorL, motorStop);
        pogobot_motor_set(motorR, motorHalf);
    } else { // si on n'a reçu aucun message
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