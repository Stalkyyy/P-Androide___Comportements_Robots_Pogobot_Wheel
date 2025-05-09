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

#define HAS_WHEEL true // Permet de choisir le cas où c'est un robot à roue, ou un robot à brosse.

#define NB_MAX_ROBOTS 2

#define POSITION_MSG 0

#define WAIT_BEFORE_ELECTION 10000000 // Temps d'attente (10 secondes) entre la formation complète de la file indienne et l'élection du leader
#define WAIT_BEFORE_WALKING 20000000 // Temps d'attente (20 secondes) entre l'élection du leader et le commencement du mouvement


/*
 * ====================================================================================
 */

// Prototypes des fonctions
int id_already_received(uint16_t id, uint16_t *tab, int taille);
void add_id_received(uint16_t id, uint16_t *tab, int taille);
void send_id(uint16_t id);

void move_front(void);
void move_left(void);
void move_right(void);
void move_stop(void);

void observe(bool detection[]);
void send_position(uint16_t type, uint16_t id);
void random_walk_leader(bool *detection);
void follow_leader(bool *detection);

bool ping_walls(void);
void walls_user_init(void);
void walls_user_step(void);

/*
 * ====================================================================================
 */

// struct du message envoyé lors du déplacement
typedef struct {
    uint16_t type; // type de message (POSITION_MSG)
    uint16_t id;
} POSITIONMSG;


// "Global" variables should be inserted within the USERDATA struct.
typedef struct {
    uint8_t data_foo[8];
    time_reference_t timer_it;
    uint16_t my_id;

    int has_leader;
    int is_leader;

    uint16_t nb_neighbours;
    uint16_t predecessor_id;
    uint16_t successor_id;
    int is_successor;
    uint16_t obstacle;
    uint8_t robot_behind;

    uint16_t nb_robots;
    uint16_t all_known_ids[NB_MAX_ROBOTS];
    time_reference_t timer;
    int timer_init;
    int start_moving;
    int stop;

    uint16_t motorLeft;
    uint16_t motorRight;

    uint8_t dirRight;
    uint8_t dirLeft;
    
    int lastDir;
    int leader_dir;
} USERDATA;

DECLARE_USERDATA(USERDATA);
REGISTER_USERDATA(USERDATA);


/*
 * ====================================================================================
 */


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


/*
 * ====================================================================================
 */


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
#ifdef SIMULATOR
    pogobot_motor_set(motorL, motorHalf);
    pogobot_motor_set(motorR, motorHalf);

    pogobot_motor_dir_set(motorL, 0);
    pogobot_motor_dir_set(motorR, 0);
#else
    if (HAS_WHEEL) {
        pogobot_motor_set(motorL, motorQuarter);
        pogobot_motor_set(motorR, mydata->motorRight);

        
        pogobot_motor_dir_set(motorL, mydata->dirLeft);
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
        pogobot_motor_set(motorL, mydata->motorLeft);
        pogobot_motor_set(motorR, motorQuarter);

        pogobot_motor_dir_set(motorL, mydata->dirLeft);
        pogobot_motor_dir_set(motorR, mydata->dirRight);
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

void observe(bool detection[]) {
    mydata->obstacle = 0;
    mydata->is_successor = 0;
    pogobot_infrared_update();
    while (pogobot_infrared_message_available()) {
        message_t msg;
        pogobot_infrared_recover_next_message(&msg);
        POSITIONMSG* received_msg = (POSITIONMSG *)msg.payload;

        if(received_msg->type == POSITION_MSG){
            int sensor_id = msg.header._receiver_ir_index;

            if (mydata->is_leader == 0){
                uint16_t sender_id =  msg.header._sender_id;
                if (sender_id == mydata->predecessor_id){
                    if (sensor_id >= 0 && sensor_id < 4) {
                        detection[sensor_id] = true;
                    }
                }
            } else {
                uint16_t sender_id =  msg.header._sender_id;
                if (sensor_id >= 0 && sensor_id < 4) {
                    detection[sensor_id] = true;
                    if(sender_id == mydata->successor_id){
                        mydata->is_successor = 1;
                    } else {
                        mydata->obstacle = 1;
                    }
                }
            }
        }
    }
}


/*
 * ====================================================================================
 */


void send_position(uint16_t type, uint16_t id) {
    POSITIONMSG msg = { type, id };
    // envoi multiple du message à différents pas de temps pour une transmission réussie (notamment dans le cas de bruitage)
    int retransmit_count = 5;
    while (retransmit_count > 0){
        static uint32_t last_sent = 0;
        if (pogobot_stopwatch_get_elapsed_microseconds(&mydata->timer_it) - last_sent > 20000) { // après 0.02 sec, semble suffisant
            pogobot_infrared_sendLongMessage_omniSpe((uint8_t *)&msg, sizeof(msg));
            last_sent = pogobot_stopwatch_get_elapsed_microseconds(&mydata->timer_it);
            retransmit_count--;
        }
    }
}


void random_walk_leader(bool *detection) {
    if (mydata->is_leader == 0 || mydata->start_moving == 0) return;

    if(mydata->leader_dir != UINT16_MAX){
        // Initialisation des capteurs
        //bool sensorFront = detection[0];
        bool sensorRight = detection[1];
        //bool sensorBack = detection[2];
        bool sensorLeft = detection[3];

        if (mydata->obstacle == 1 && sensorRight){ // obstacle à droite
            move_left();
            send_position(POSITION_MSG, mydata->my_id);
            return;
        } else if (mydata->obstacle == 1 && sensorLeft) { // obstacle à gauche
            move_right();
            send_position(POSITION_MSG, mydata->my_id);
            return;
        } else if (mydata->is_successor == 1 && (sensorRight || sensorLeft)){
            move_front();
        } else if (mydata->is_successor == 0){ // plus de successeur derrière
            move_stop();
            mydata->stop = 1;
            send_position(POSITION_MSG, mydata->my_id);
            return;
        } 
    }

    if(pogobot_ticks % 145 == 0){ // 45 tours de user_step pendant 3 secondes
        mydata->leader_dir = rand() % 3; // 0: tout droit, 1: droite, 2: gauche

        if(mydata->leader_dir == 0){ // tout droit
            mydata->lastDir = mydata->leader_dir;
            move_front();
        } else if (mydata->leader_dir == 1){ // vers la droite
            if (mydata->lastDir == mydata->leader_dir){ // si dernière direction était vers la droite, il va à gauche
                mydata->lastDir = 2;
                move_left();
            } else {
                mydata->lastDir = mydata->leader_dir;
                move_right();
            }
        } else if(mydata->leader_dir == 2){ // vers la gauche
            if (mydata->lastDir == mydata->leader_dir){ // si dernière direction était vers la gauche, il va à droite
                mydata->lastDir = 1;
                move_right();
            } else {
                mydata->lastDir = mydata->leader_dir;
                move_left();
            }
        }
    }

    else {
        if(mydata->stop == 1){
            move_front();
            mydata->stop = 0;
        }
        send_position(POSITION_MSG, mydata->my_id);
    }
}


void follow_leader(bool *detection) {
    if (mydata->is_leader == 1) return;

    // Initialisation des capteurs
    bool sensorFront = detection[0];
    bool sensorRight = detection[1];
    bool sensorBack = detection[2];
    bool sensorLeft = detection[3];

    if (sensorRight){
        move_right();
    }
    else if (sensorLeft){
        move_left();
    }
    else if (sensorFront){
        move_front();
    } else if (mydata->robot_behind == 1 && !sensorBack){
        move_stop();
    }
    send_position(POSITION_MSG, mydata->my_id);

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


    mydata->my_id = pogobot_helper_getid();
    mydata->is_leader = 0;
    mydata->has_leader = 0;
    mydata->predecessor_id = UINT16_MAX;
    mydata->successor_id = UINT16_MAX;
    mydata->is_successor = 0;
    mydata->obstacle = 0;
    mydata->robot_behind = 0;
    mydata->nb_robots = 1;
    mydata->start_moving = 0;
    mydata->lastDir = 0;
    mydata->leader_dir = UINT16_MAX;
    mydata->timer_init = 0;
    mydata->stop = 0;

    pogobot_stopwatch_reset(&mydata->timer);

    mydata->all_known_ids[0] = mydata->my_id;
    for (uint8_t i = 1; i < NB_MAX_ROBOTS; i++){
        mydata->all_known_ids[i] = UINT16_MAX;
    }

    pogobot_led_setColor(120, 60, 0); // jaune avant et pendant l'élection du leader
}


/*
 * ====================================================================================
 */


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
                    mydata->nb_robots++;
                }
                send_id(msg.payload[0]); 
                send_id(mydata->my_id);             
            }
        }

        else if (mydata->nb_robots == NB_MAX_ROBOTS){
            if(mydata->timer_init == 0){
                pogobot_timer_init(&mydata->timer, WAIT_BEFORE_ELECTION);
                mydata->timer_init = 1;
            } else {
                if(pogobot_timer_has_expired(&mydata->timer)) { // si le timer a  expiré, établissement des rôles
                    if((mydata->predecessor_id != UINT16_MAX && mydata->robot_behind == 0) || (mydata->predecessor_id != UINT16_MAX && mydata->robot_behind == 1)){
                        pogobot_led_setColor(0, 0, 255); // bleu follower
                        mydata->has_leader = 1;
                        mydata->timer_init = 0;
                    } else if(mydata->predecessor_id == UINT16_MAX && mydata->robot_behind == 1){
                        pogobot_led_setColor(0, 255, 0); // vert leader
                        mydata->is_leader = 1;
                        mydata->has_leader = 1;
                        mydata->timer_init = 0;
                    }
                    pogobot_infrared_clear_message_queue();
                    pogobot_infrared_update();
                    
                } else {
                    pogobot_infrared_update();
                    if (pogobot_infrared_message_available()){
                        message_t msg;
                        pogobot_infrared_recover_next_message(&msg);

                        // détection des robots de devant et derrière pour établir le sens de la file indienne
                        uint8_t direction = msg.header._receiver_ir_index;
                        if (direction == 0){
                            mydata->predecessor_id = msg.header._sender_id;
                            //printf("détecté en face %d : id %d\n", direction, mydata->predecessor_id);
                        } else if (direction == 2){
                            //printf("détecté derrière %d\n", direction);
                            mydata->robot_behind = 1;
                            mydata->successor_id = msg.header._sender_id;
                        }
                        send_id(msg.payload[0]);
                    }
                    send_id(mydata->my_id);
                }
            }
        }
    }

    else if (mydata->has_leader == 1 && mydata->nb_robots == NB_MAX_ROBOTS){
        // le leader attends quelques temps avant de commencer à se déplacer pour que tous les robots soient prêts à le suivre
        if(mydata->is_leader == 1 && mydata->start_moving == 0){
            if(mydata->timer_init == 0){
                pogobot_timer_init(&mydata->timer, WAIT_BEFORE_WALKING);
                mydata->timer_init = 1;
            } else {
                if(pogobot_timer_has_expired(&mydata->timer)){
                    mydata->start_moving = 1;
                    mydata->timer_init = 0;
                }
            }
        }

        bool detection[4] = {false, false, false, false};
        observe(detection);
        random_walk_leader(detection);
        follow_leader(detection);
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
