#include "pogobase.h"
#include <stdlib.h>
#include <time.h>

#define HAS_WHEEL false

#define NB_MAX_ROBOTS 2

#define ID_EXTREMITY 1
#define ID_LEADER 2
#define POSITION_MSG 3

#define WAIT_BEFORE_ELECTION 5000000
#define WALK_IN_DIRECTION_TIME 3000000 // (en microsec) durée de déplacement dans une direction donnée (pendant 3 sec max)
//#define SAFE_DISTANCE  JE SAIS PAS QUOI METTRE POUR L'INSTANT mais une distance de sécurité entre les robots pour éviter les collsiions ou qu'ils soient trop près


// Prototypes des fonctions
void detect_neighbours(void);
void send_id_extremity(void);
void election_leader(void);
void set_line_order(void);
void send_id(uint16_t id);
void send_position(uint16_t type, uint16_t id, uint16_t ml, uint16_t mr, uint16_t dir);
void random_walk_leader(void);
void follow_leader(void);

// struct message pour partager les ids des robots (aux extrémités et du leader)
typedef struct {
    uint16_t type; // type de message (ID_EXTREMITY ou ID_LEADER)
    uint16_t id;
} MSG;

#define MSG_SIZE sizeof(MSG) // Number of bytes

typedef union msg_template {
    uint8_t msg_array[MSG_SIZE];
    MSG msg_values;
} message;

// struct du message concernant position du leader à suivre
typedef struct {
    uint16_t type; // type de message (POSITION_MSG)
    uint16_t id; // id du sender ici
    uint16_t motorL;
    uint16_t motorR;
    uint16_t dir;
} POSITIONMSG;

// "Global" variables should be inserted within the USERDATA struct.
typedef struct {
    uint8_t data_foo[8];
    time_reference_t timer_it;
    uint16_t my_id;

    int has_leader;
    int is_leader;

    uint16_t nb_neighbours; // max 2 voisins car disposition en file indienne
    uint16_t neighbours_ids[2];
    uint16_t predecessor_id; // id du robot à suivre dans la file indienne

    uint16_t nb_robots;
    uint16_t all_known_ids[NB_MAX_ROBOTS]; // tous les id (des autres robots et de lui-même) que le robot connait (taille 20 car max 20 robots selon le sujet)
    time_reference_t timer;
    int timer_init; // pour initialiser le timer qu'une seule fois
    int start_election;
    int start_moving;

    uint16_t motorLeft;
    uint16_t motorRight;
    uint8_t dirRight;
    uint8_t dirLeft;
    int lastDir;
} USERDATA;

DECLARE_USERDATA(USERDATA);
REGISTER_USERDATA(USERDATA);

void user_init(void) {
#ifndef SIMULATOR
    printf("setup ok\n");
#endif

    pogobot_stopwatch_reset(&mydata->timer_it);
    main_loop_hz = 45; // mettre 45 ??? remettre 60 ???????
    max_nb_processed_msg_per_tick = 0;
    msg_rx_fn = NULL;
    msg_tx_fn = NULL;
    error_codes_led_idx = 3;
    pogobot_infrared_set_power(3);

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
    } else{
        mydata->motorLeft = motorHalf;
        mydata->motorRight = motorHalf;
    }

    mydata->my_id = pogobot_helper_getid();
    mydata->is_leader = 0;
    mydata->has_leader = 0;
    mydata->predecessor_id = -1;
    mydata->nb_neighbours = 0;
    mydata->nb_robots = 1;
    mydata->start_election = 0;
    mydata->start_moving = 0;
    mydata->lastDir = 0;
    mydata->timer_init = 0;
    pogobot_stopwatch_reset(&mydata->timer);

    for (uint8_t i = 0; i < 2; i++){
        mydata->neighbours_ids[i] = UINT16_MAX;
    }

    mydata->all_known_ids[0] = mydata->my_id; // pas nécessaire si on vérifie si id reçu != my id mais bon, pq pas
    for (uint8_t i = 1; i < NB_MAX_ROBOTS; i++){
        mydata->all_known_ids[i] = UINT16_MAX;
    }

    pogobot_led_setColor(120, 60, 0); // jaune avant et pendant l'élection du leader
}

int id_already_received(uint16_t id, uint16_t *tab, int taille){
    for (uint8_t i = 0; i < taille; i++){
        if(tab[i] == id){
            return 1;
        }
    }
    return 0;
}

void add_id_received(uint16_t id, uint16_t *tab, int taille){
    for (uint8_t i = 0; i < taille; i++){
        if(tab[i] == UINT16_MAX){
            tab[i] = id;
            return;
        }
    }
}

void send_id(uint16_t id){
    uint8_t msg[1];
    msg[0] = id;
    pogobot_infrared_sendLongMessage_omniSpe(msg, sizeof(msg));
}

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

        pogobot_motor_dir_set(motorL, ((mydata->dirLeft + 1) % 2));
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
        pogobot_motor_dir_set(motorR, ((mydata->dirRight + 1) % 2));
    } else {
        pogobot_motor_set(motorL, motorHalf);
        pogobot_motor_set(motorR, motorStop);
    }
}

void move_stop(void) {
    pogobot_motor_set(motorL, motorStop);
    pogobot_motor_set(motorR, motorStop);
}

void detect_neighbours(void) {
    send_id(mydata->my_id);

    // time_reference_t t;
    // pogobot_stopwatch_reset(&t);
    // && pogobot_stopwatch_get_elapsed_microseconds(&t) < 5000000

    pogobot_infrared_update();
    while (pogobot_infrared_message_available()) {
        message_t msg;
        pogobot_infrared_recover_next_message(&msg);
        // uint16_t sender_id = msg.header._sender_id;
        if (msg.payload[0] != mydata->my_id && id_already_received(msg.payload[0], mydata->neighbours_ids, 2)==0 && mydata->nb_neighbours<=1){
        // if (sender_id != mydata->my_id && id_already_received(sender_id)==0 && mydata->nb_neighbours<=1){
            add_id_received(msg.payload[0], mydata->neighbours_ids, 2);
            // add_id_received(sender_id);
            mydata->nb_neighbours++;
            // printf("Robot %d a trouvé voisin %d!\n", mydata->my_id, sender_id);
            //printf("Robot %d a trouvé voisin %d!\n", mydata->my_id, msg.payload[0]);
        }
        send_id(mydata->my_id); //renvoit son id au cas où non reçu
    }
}

void send_id_extremity(void) {
    if (mydata->nb_neighbours == 1) {
        uint8_t data[MSG_SIZE];
        message msg;

        msg.msg_values.type = ID_EXTREMITY;
        msg.msg_values.id = mydata->my_id;

        for ( uint16_t i = 0; i != MSG_SIZE; i++ )
            data[i] = msg.msg_array[i];
        
        pogobot_infrared_sendLongMessage_omniSpe((uint8_t *)(data), MSG_SIZE); 
    }
}

void transmission_msg(int type, uint16_t id){
    uint8_t data[MSG_SIZE];
    message msg;

    msg.msg_values.type = type;
    msg.msg_values.id = id;

    for ( uint16_t i = 0; i != MSG_SIZE; i++ )
        data[i] = msg.msg_array[i];
    
    if (type == ID_LEADER){ // envoi du message plusieurs fois (5 fois max à intervalle de tps régulier) car non fait une fois les rôles établis
        int retransmit_count = 5;
        while (retransmit_count > 0){
            static uint32_t last_sent = 0;
            if (pogobot_stopwatch_get_elapsed_microseconds(&mydata->timer_it) - last_sent > 30000) { //après 0.03 sec, semble suffisant
                pogobot_infrared_sendLongMessage_omniSpe((uint8_t *)(data), MSG_SIZE); 
                last_sent = pogobot_stopwatch_get_elapsed_microseconds(&mydata->timer_it);
                retransmit_count--;
            }
        }
    }
    else {
        pogobot_infrared_sendLongMessage_omniSpe((uint8_t *)(data), MSG_SIZE);  
    }
}

void election_leader(void) {
    pogobot_infrared_update();
    while (pogobot_infrared_message_available()) {
        message_t msg;
        pogobot_infrared_recover_next_message(&msg);
        message received_msg;

        for (uint16_t i = 0; i != MSG_SIZE; i++)
            received_msg.msg_array[i] = msg.payload[i];

        if (received_msg.msg_values.type == ID_EXTREMITY){
            uint16_t id = received_msg.msg_values.id;
            if (mydata->nb_neighbours == 1) {
                if (mydata->my_id == id){
                    break;
                }
                else if (mydata->my_id > id){
                    mydata->predecessor_id = msg.header._sender_id;
                    pogobot_led_setColor(0, 0, 255); // bleu pour follower
                    //printf("PAS LEADER car: %d VS %d", mydata->my_id, id);
                    //printf("Robot %d a pour prédecesseur : %d\n", mydata->my_id, mydata->predecessor_id);
                    mydata->has_leader = 1;
                }
                else if (mydata->my_id < id) {
                    mydata->is_leader = 1;
                    pogobot_led_setColor(0, 255, 0); // vert pour le leader
                    //printf("LEADER car myid %d < autre id %d donc envoi message leader %d\n", mydata->my_id, id, mydata->my_id);
                    transmission_msg(ID_LEADER, mydata->my_id);
                    mydata->has_leader = 1;
                }
            }
            else if (mydata->nb_neighbours > 1){
                transmission_msg(ID_EXTREMITY, id);
            }
        }
    }
}

void set_line_order(void) {
    pogobot_infrared_update();
    while (pogobot_infrared_message_available()) {
        message_t msg;
        pogobot_infrared_recover_next_message(&msg);
        message received_msg;

        for (uint16_t i = 0; i != MSG_SIZE; i++)
            received_msg.msg_array[i] = msg.payload[i];

        if (received_msg.msg_values.type == ID_LEADER){
            if (mydata->has_leader == 0) {
                //if (id_already_received(msg.header._sender_id, mydata->neighbours_ids, 2)==1 ){ // if peut-être pas nécessaire mais on sait jamais...
                mydata->predecessor_id = msg.header._sender_id;
                //printf("SET LEADER : %d a pour prédecesseur : %d\n", mydata->my_id, mydata->predecessor_id);
                pogobot_led_setColor(0, 0, 255); // bleu pour le follower
                transmission_msg(ID_LEADER, received_msg.msg_values.id);
                mydata->has_leader = 1;
                // }
                // else
                //     transmission_msg(ID_LEADER, received_msg.msg_values.id);
            }
        }
    }
}


void send_position(uint16_t type, uint16_t id, uint16_t ml, uint16_t mr, uint16_t dir) {
    POSITIONMSG msg = { type, id, ml, mr, dir };
    pogobot_infrared_sendLongMessage_omniSpe((uint8_t *)&msg, sizeof(msg));
}

void random_walk_leader(void) {
    if (mydata->is_leader == 0) return;

    int directions[] = { 0, 1, 2, 3 }; // 0: tout droit, 1: droite, 2: gauche, 3: stop
    int random_index = rand() % 4;
    int random_direction = directions[random_index];
    //printf("DIRECTION LEADER %d\n", random_direction);
    if(random_direction == 0){ // tout droit
        mydata->lastDir = random_direction;
        move_front();
    } else if (random_direction == 1){ // vers la droite
        if (mydata->lastDir == random_direction){ // si dernière direction était vers la droite, va à gauche
            mydata->lastDir = 2;
            move_left();
        } else {
            mydata->lastDir = random_direction;
            move_right();
        }
    } else if(random_direction == 2){ // vers la gauche
        if (mydata->lastDir == random_direction){ // si dernière direction était vers la gauche, va à droite
            mydata->lastDir = 1;
            move_right();
        } else {
            mydata->lastDir = random_direction;
            move_left();
        }
    } else { // random_direction == 3 -> ARRÊT
        if (mydata->lastDir == 3){ // si dernière direction était stop, va tout droit
            mydata->lastDir = 0;
            move_front();
        } else {
            mydata->lastDir = 3;
            move_stop();
        }
    }

    if(random_direction == 0 || random_direction == 3){
        time_reference_t timer;
        // pogobot_timer_init(&timer, WALK_IN_DIRECTION_TIME);
        pogobot_stopwatch_reset(&timer);
        // while (!pogobot_timer_has_expired(&timer)){
        while(pogobot_stopwatch_get_elapsed_microseconds(&timer) < WALK_IN_DIRECTION_TIME){
            // printf("TIMER NON FINI %d", pogobot_timer_get_remaining_microseconds(&timer));
            //printf("TIMER NON FINI %d -> direction  %d\n", pogobot_stopwatch_get_elapsed_microseconds(&timer), random_direction);
            send_position(POSITION_MSG, mydata->my_id, mydata->motorLeft, mydata->motorRight, random_direction);
        }
    }
    else{ // si tourne à droite ou à gauche, envoi simplement du msg (nécessité d'envoyer plusieurs fois ou pas?? à voir irl)
        send_position(POSITION_MSG, mydata->my_id, mydata->motorLeft, mydata->motorRight, random_direction);
    }
}

void follow_leader(void) {
    if (mydata->is_leader == 1) return;

    pogobot_infrared_update();

    while (pogobot_infrared_message_available()) {
        message_t msg;
        pogobot_infrared_recover_next_message(&msg);
        POSITIONMSG* received_msg = (POSITIONMSG *)msg.payload;
        //printf("Message position reçu: %d, %d, %d\n", received_msg->type, received_msg->id, received_msg->dir);

        if(received_msg->type == POSITION_MSG){
            int direction = msg.header._receiver_ir_index;

            if (received_msg->id == mydata->predecessor_id) {
                //printf("DIRECTION DU PRED %d\n", received_msg->dir);
                if(received_msg->dir == 3){ // si le prédecesseur est arrêté, s'arrête aussi
                    //printf("FOLLOWER STOP\n");
                    move_stop();
                    send_position(POSITION_MSG, mydata->my_id, received_msg->motorL, received_msg->motorR, received_msg->dir);
                } else {
                    //printf("FOLLOWER WALK\n", direction);
                    if (direction == 0){ //si predecesseur devant est détecté par en face 
                        // AJOUTER PEUT ETRE direction == 2 -> si le sens de la file indienne est inversée selon le leader élu
                        move_front();
                        // pogobot_motor_set(motorL, motorHalf);
                        // pogobot_motor_set(motorR, motorHalf);
                        // pogobot_motor_set(motorL, received_msg->motorL);
                        // pogobot_motor_set(motorR, received_msg->motorR);
                    }
                    else if(direction == 1){ // si prédecesseur détecté à droite
                        move_right();
                        // pogobot_motor_set(motorL, motorHalf);
                        // pogobot_motor_set(motorR, motorQuarter);
                    }
                    else if(direction == 3){ // si prédecesseur détecté à gauche
                        move_left();
                        // pogobot_motor_set(motorL, motorQuarter);
                        // pogobot_motor_set(motorR, motorHalf);
                    }

                    // AJOUTER CONDITION SI ROBOT DE DEVANT S'ARRETE!!!!!!
                    // distance de sécurité? ou envoi de msg pour avertir cela? puissance ir ??

                    // pogobot_motor_set(motorL, received_msg->motorL);
                    // pogobot_motor_set(motorR, received_msg->motorR);
                    send_position(POSITION_MSG, mydata->my_id, received_msg->motorL, received_msg->motorR, received_msg->dir);
                }
            }
        }
    }
}

void user_step(void) {
    if (mydata->has_leader == 0){ // en attendant la formation complète de la file indienne
        if (mydata->nb_robots < NB_MAX_ROBOTS){
            send_id(mydata->my_id);
            pogobot_infrared_update();
            while (pogobot_infrared_message_available() && mydata->nb_robots < NB_MAX_ROBOTS) {
                message_t msg;
                pogobot_infrared_recover_next_message(&msg);
                if (msg.payload[0] != mydata->my_id && id_already_received(msg.payload[0], mydata->all_known_ids, NB_MAX_ROBOTS)==0){
                    add_id_received(msg.payload[0], mydata->all_known_ids, NB_MAX_ROBOTS);
                    send_id(msg.payload[0]);
                    send_id(mydata->my_id);// renvoit son id au cas où nouveau robot ajouté à la file
                    mydata->nb_robots++;
                    //printf("Robot %d a trouvé robot %d -> nb robots = %d!\n", mydata->my_id, msg.payload[0], mydata->nb_robots);
                }
                else{
                    send_id(msg.payload[0]); // envoit quand même le message pour les autres robots qui ne l'ont pas encore reçu
                    send_id(mydata->my_id); // renvoit son id au cas où nouveau robot ajouté à la file
                }                
            }
            // pogobot_infrared_clear_message_queue(); // peut être pas nécessaire de supprimer ces msg
            // pogobot_infrared_update();
        }

        else if (mydata->nb_robots == NB_MAX_ROBOTS && mydata->start_election == 0){
            // time_reference_t t;
            // pogobot_stopwatch_reset(&t);
            if(mydata->timer_init == 0){
                pogobot_timer_init(&mydata->timer, WAIT_BEFORE_ELECTION);
                mydata->timer_init = 1;
            } else {
                if(pogobot_timer_has_expired(&mydata->timer)) { // si le timer a  expiré
                    pogobot_infrared_clear_message_queue();
                    pogobot_infrared_update();
                    pogobot_led_setColor(255, 255, 0);
                    //mydata->timer_init = 0;
                    mydata->start_election = 1;

                    // pogobot_infrared_update();
                    // printf("TIMER NON FINI\n");
                    // if (pogobot_infrared_message_available()){
                    //     message_t msg;
                    //     pogobot_infrared_recover_next_message(&msg);
                    //     send_id(msg.payload[0]);
                    // }
                    // send_id(mydata->my_id);
                } else {
                    pogobot_infrared_update();
                    printf("TIMER NON FINI\n");
                    if (pogobot_infrared_message_available()){
                        message_t msg;
                        pogobot_infrared_recover_next_message(&msg);
                        send_id(msg.payload[0]);
                    }
                    send_id(mydata->my_id);
                }
            }

            // OU ALORS n'a pas le temps de recevoir un message du coup false pour la 1ère condition (peut être que dans le cas n=2)
            //while(pogobot_stopwatch_get_elapsed_microseconds(&t) < 10000000) et if(pogobot_infrared_message_available())
            // while (pogobot_stopwatch_get_elapsed_microseconds(&t) < 10000000) { // renvoi de msg pendant 5 sec
            //     pogobot_infrared_update();
            //     if (pogobot_infrared_message_available()){
            //         message_t msg;
            //         pogobot_infrared_recover_next_message(&msg);
            //         //printf("ELSE IF Temps intervalle %d", pogobot_stopwatch_get_elapsed_microseconds(&t));
            //         send_id(msg.payload[0]);
            //     }
            //     send_id(mydata->my_id);
            // }
            // pogobot_infrared_clear_message_queue();
            // pogobot_infrared_update();
            // mydata->start_election = 1;
        }

        else if (mydata->nb_robots == NB_MAX_ROBOTS && mydata->start_election == 1){
            if (mydata->nb_neighbours == 0){
                detect_neighbours();
            }
            send_id_extremity();
            election_leader();
            set_line_order();
        }
    }

    // ajout d'un timer de 5sec après les rôles établis (même si ça fait trop de timer...) + condition sur random_walk
    // else if (mydata->has_leader == 1 && mydata->nb_robots == NB_MAX_ROBOTS && mydata->start_election == 1){
    //     printf("Dans else if %d\n", mydata->start_moving);
    //     if(mydata->start_moving == 0){
    //         printf("dans if \n");
    //         //time_reference_t t;
    //         //pogobot_stopwatch_reset(&t);
    //         if(mydata->timer_init == 0){
    //             pogobot_timer_init(&mydata->timer, 5000000);
    //             mydata->timer_init =1;
    //         } else{
    //             pogobot_timer_wait_for_expiry(&mydata->timer);
    //             mydata->start_moving = 1;
    //         }
    //         // while(pogobot_stopwatch_get_elapsed_microseconds(&t) < 5000000){ //  attente de 5sec
    //         //     printf("Bloqué\n");
    //         // }
    //         // mydata->start_moving = 1;
    //     } else {
    //         printf("Enfin dans le else pour move \n");
    //         random_walk_leader();
    //         follow_leader();
    //     }
    // }

    else if (mydata->has_leader == 1 && mydata->nb_robots == NB_MAX_ROBOTS && mydata->start_election == 1){
        random_walk_leader();
        follow_leader();
    }
}

int main(void) {
    pogobot_init();
#ifndef SIMULATOR
    printf("init ok\n");
#endif
    pogobot_start(user_init, user_step);
    return 0;
}