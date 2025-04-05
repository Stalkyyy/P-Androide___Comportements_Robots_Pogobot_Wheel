// Main include for pogobots, both for real robots and for simulations
#include "pogobase.h"

#define INFRARED_POWER 3 // 1, 2, 3

#define MAX_ROBOTS 5

// "Global" variables should be inserted within the USERDATA struct.
// /!\  In simulation, don't declare non-const global variables outside this struct, elsewise they will be shared among all agents (and this is not realistic).
typedef struct {
    // Put all global variables you want here.
    uint8_t data_foo[8];
    time_reference_t timer_it;
    uint8_t suitDeja; // indique si on suit déjà qlq (plus vraiment utile vu qu'on stocke les ids des robots mtn)
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

    mydata->suitDeja = 0; // au départ, le robot ne suit personne, c'est pour être sûr que l'id est vraiment l'id d'un robot et pas la valeur initiale
    //mydata->id_robots_suivis = {0}; // id du robot qu'on suit 
    mydata->nb_robots_suivis = 0;
}


// Step function. Called continuously at each step of the pogobot main loop
void user_step(void) {

    // Transmission d'un message
    uint8_t msg_envoi[MAX_ROBOTS+2];
    msg_envoi[0] = mydata->suitDeja; // indique si le robot suit déjà qlq (pas d'inspi pour le nom j'avoue)
    msg_envoi[1] = mydata->nb_robots_suivis;
    for(int i=0; i<MAX_ROBOTS; i++){
        msg_envoi[i+2] = mydata->id_robots_suivis[i];
    }
    
    pogobot_infrared_sendLongMessage_omniSpe(msg_envoi, sizeof(msg_envoi));

    // Réception d'un message et décision du mouvement à faire
    uint8_t move_id = 5; // valeur de 5 car ça ne correspond à aucune IR direction
    pogobot_infrared_update();

    uint8_t dir[4] = {0}; // compteur des senseurs des directions reçues
    uint8_t id_robots_dir[4][MAX_ROBOTS] = {0}; // stockage des ids des robots détectés en fct des directions
    
    while(pogobot_infrared_message_available()>=1){
        message_t msg;
        pogobot_infrared_recover_next_message(&msg);

        // si on détecte un robot
        if(msg.header._packet_type == ir_t_user){
            uint8_t skip = 0;

            // si c'est l'un des robots qui nous suit déjà alors on ne l'ignore
            for(int i=0; i<msg.payload[1]; i++){
                if(msg.payload[i+2] == pogobot_helper_getid()){
                    skip = 1;
                    break;
                }
            }
            if(skip){
                continue;
            }

            // on stocke l'id du robot qui a envoyé la direction
            id_robots_dir[msg.header._receiver_ir_index][dir[msg.header._receiver_ir_index]] = msg.header._sender_id;
            
            // on stocke le nombre de robots ayant envoyé cette direction
            dir[msg.header._receiver_ir_index] ++;
        } 
        // si on détecte autre / un mur (à implémenter selon ce que va donner wall_allignment)
        else {
            printf("Mur !\n");
        }
    }

    // on regarde si le robot a vu des voisins qu'il peut suivre
    uint8_t suitPas = 0;
    for(int i=0; i<4; i++){
        if(dir[i] > 0){
            suitPas = 1;
        }
    }

    // s'il a trouvé qlq à suivre, il s'adapte à la direction majoritaire
    if(suitPas == 1){

        // on cherche la direction où y a le plus de robots
        uint8_t nb_robots_max = 0;
        for(int i=0; i<4; i++){
            if(nb_robots_max < dir[i]){
                nb_robots_max = dir[i];
            }
        }

        // on vérifie si y a une égalité entre plusieurs directions
        uint8_t dir_egal[4] = {0};
        uint8_t cpt_egalite = 0;
        for(int i=0; i<4; i++){
            if(nb_robots_max == dir[i]){
                dir_egal[cpt_egalite] = i;
                cpt_egalite++;
            }
        }

        // si y a égalité, on prend l'une des directions au hasard
        if(cpt_egalite > 1){
            uint8_t idx = rand() % cpt_egalite;
            move_id = dir_egal[idx];
        } else {
            move_id = dir_egal[0];
        }

        // on récupère les ids des voisins qu'on suit 
        memcpy(mydata->id_robots_suivis, id_robots_dir[move_id], MAX_ROBOTS * sizeof(uint8_t));
        mydata->suitDeja = 1; // plus vraiment utile ça je crois
        mydata->nb_robots_suivis = dir[move_id];
        pogobot_led_setColor(255, 0, 0);

    } else { // s'il n'a vu personne d'intérêt, il continue sa vie
        pogobot_led_setColor(0, 255, 0); 
    }


    // Mouvement 

    // s'il le détecte à droite alors il va à droite 
    // et s'il le détecte en face il tourne un peu à droite pour éviter de le foncer dedans et ensuite le suivre
    if (move_id == 1 || move_id == 0) { 
        pogobot_motor_set(motorL, motorHalf);
        pogobot_motor_set(motorR, motorStop);
    } else if (move_id == 3) { // s'il le détecte à gauche alors il tourne à gauche
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