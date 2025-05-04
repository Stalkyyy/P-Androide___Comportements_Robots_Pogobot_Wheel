#include "pogobase.h"
#include <stdlib.h>
#include <time.h>

#define HAS_WHEEL false

#define NB_MAX_ROBOTS 2

#define POSITION_MSG 0

#define WAIT_BEFORE_ELECTION 10000000 // 10 secondes d'attente (après la file au complet, et avant l'élection du leader)
#define WAIT_BEFORE_WALKING 20000000 // 20 sec d'attente avant que le leader commence à se déplacer
#define WALK_IN_DIRECTION_TIME 3000000 // durée de déplacement dans une direction donnée (pendant 3 sec max)


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

// struct du message concernant position du leader à suivre
typedef struct {
    uint16_t type; // type de message (POSITION_MSG)
    uint16_t id; // id du sender ici
    uint16_t dir;
} POSITIONMSG;

// "Global" variables should be inserted within the USERDATA struct.
typedef struct {
    uint8_t data_foo[8];
    time_reference_t timer_it;
    uint16_t my_id;

    int has_leader;
    int is_leader;

    //uint8_t receptions[2];

    uint16_t nb_neighbours; // max 2 voisins car disposition en file indienne
    uint16_t neighbours_ids[2];
    uint16_t predecessor_id; // id du robot à suivre dans la file indienne
    uint8_t robot_behind; // 0 si faux, 1 si vrai

    uint16_t nb_robots;
    uint16_t all_known_ids[NB_MAX_ROBOTS]; // tous les id (des autres robots et de lui-même) que le robot connait (taille 20 car max 20 robots selon le sujet)
    time_reference_t timer;
    int timer_init; // pour initialiser le timer qu'une seule fois
    int start_moving;

    uint16_t motorLeft;
    uint16_t motorRight;
    uint8_t dirRight;
    uint8_t dirLeft;
    int lastDir;
    int leader_dir; // direction du leader choisie aléatoirement (valeur conservée pendant 3 sec)
} USERDATA;

DECLARE_USERDATA(USERDATA);
REGISTER_USERDATA(USERDATA);

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
    mydata->predecessor_id = UINT16_MAX;
    mydata->robot_behind = 0;
    mydata->nb_robots = 1;
    mydata->start_moving = 0;
    mydata->lastDir = 0;
    mydata->leader_dir = UINT16_MAX;
    mydata->timer_init = 0;
    pogobot_stopwatch_reset(&mydata->timer);

    mydata->all_known_ids[0] = mydata->my_id;
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
        pogobot_motor_set(motorL, motorQuarter);
        pogobot_motor_set(motorR, mydata->motorRight);

        pogobot_motor_dir_set(motorL, mydata->dirLeft);
        pogobot_motor_dir_set(motorR, mydata->dirRight);
    } else {
        pogobot_motor_set(motorL, motorStop);
        pogobot_motor_set(motorR, motorHalf);
    }
}

void move_right(void) {
    if (HAS_WHEEL) {
        pogobot_motor_set(motorL, mydata->motorLeft);
        pogobot_motor_set(motorR, motorQuarter);

        pogobot_motor_dir_set(motorL, mydata->dirLeft);
        pogobot_motor_dir_set(motorR, mydata->dirRight);
    } else {
        pogobot_motor_set(motorL, motorHalf);
        pogobot_motor_set(motorR, motorStop);
    }
}

void move_stop(void) {
    pogobot_motor_set(motorL, motorStop);
    pogobot_motor_set(motorR, motorStop);
}

void observe(bool detection[]) {
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
                if (sensor_id >= 0 && sensor_id < 4) {
                    detection[sensor_id] = true;
                }
            }
        }
    }
}



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

    // après avoir commencé le mouvement seulement, vérification d'obstacle et de perte de successeur
    if(mydata->leader_dir != UINT16_MAX){
        bool sensorRight = detection[1];
        bool sensorBack = detection[2];
        bool sensorLeft = detection[3];

        if (sensorRight && sensorBack){ // obstacle à droite
            move_left();
            send_position(POSITION_MSG, mydata->my_id);  // envoie quand même msg pour son successeur si le suit tjr
            // pogobot_infrared_clear_message_queue();
            // pogobot_infrared_update();
            return;
        } else if (sensorLeft && sensorBack) { // obstacle à gauche
            move_right();
            send_position(POSITION_MSG, mydata->my_id);
            // pogobot_infrared_clear_message_queue();
            // pogobot_infrared_update();
            return;
        } else if (!sensorBack){ // si ne reçoit plus de message de son successeur, alors ARRÊT
            move_stop();
            send_position(POSITION_MSG, mydata->my_id); // envoi quand même un message au cas où son successeur aurait retrouvé la trace
            return;
        }
    }

    if(mydata->timer_init == 0){
        mydata->leader_dir = rand() % 3; // 0: tout droit, 1: droite, 2: gauche

        if(mydata->leader_dir == 0){ // tout droit
            mydata->lastDir = mydata->leader_dir;
            move_front();
        } else if (mydata->leader_dir == 1){ // vers la droite
            if (mydata->lastDir == mydata->leader_dir){ // si dernière direction était vers la droite, va à gauche
                mydata->lastDir = 2;
                move_left();
            } else {
                mydata->lastDir = mydata->leader_dir;
                move_right();
            }
        } else if(mydata->leader_dir == 2){ // vers la gauche
            if (mydata->lastDir == mydata->leader_dir){ // si dernière direction était vers la gauche, va à droite
                mydata->lastDir = 1;
                move_right();
            } else {
                mydata->lastDir = mydata->leader_dir;
                move_left();
            }
        }
        pogobot_timer_init(&mydata->timer, WALK_IN_DIRECTION_TIME);
        mydata->timer_init = 1;
    }

    if(mydata->timer_init == 1){
        if(pogobot_timer_has_expired(&mydata->timer)){ // timer expiré, changement de direction au prochain tour de user_step()
            mydata->timer_init = 0;
        }
        send_position(POSITION_MSG, mydata->my_id);
    }
}

void follow_leader(bool *detection) {
    if (mydata->is_leader == 1) return;

    bool sensorFront = detection[0];
    bool sensorRight = detection[1];
    bool sensorBack = detection[2];
    bool sensorLeft = detection[3];

    if (sensorFront && sensorRight){
        move_right();
    }
    else if (sensorFront && sensorLeft){
        move_left();
    }
    else if (sensorFront){
        move_front();
    } else if (mydata->robot_behind == 1 && !sensorBack){
        move_stop();
    }
    send_position(POSITION_MSG, mydata->my_id);

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
                        mydata->timer_init = 0; // au cas où on veut réutiliser le timer plus tard
                    } else if(mydata->predecessor_id == UINT16_MAX && mydata->robot_behind == 1){
                        pogobot_led_setColor(0, 255, 0); // vert leader
                        mydata->is_leader = 1;
                        mydata->has_leader = 1;
                        mydata->timer_init = 0; // au cas où on veut réutiliser le timer plus tard
                    }
                    pogobot_infrared_clear_message_queue();
                    pogobot_infrared_update();
                    
                } else {
                    pogobot_infrared_update();
                    if (pogobot_infrared_message_available()){
                        message_t msg;
                        pogobot_infrared_recover_next_message(&msg);
                        // détection des robots de devant + derrière
                        uint8_t direction = msg.header._receiver_ir_index;
                        if (direction == 0){
                            mydata->predecessor_id = msg.header._sender_id;
                            printf("direction en face %d : id %d\n", direction, mydata->predecessor_id);
                        } else if (direction == 2){
                            printf("direction derrière %d\n", direction);
                            mydata->robot_behind = 1;
                        }
                        send_id(msg.payload[0]);
                    }
                    send_id(mydata->my_id);
                }
            }
        }
    }

    else if (mydata->has_leader == 1 && mydata->nb_robots == NB_MAX_ROBOTS){
        if(mydata->is_leader == 1 && mydata->start_moving == 0){ // le leader attend au moins 15 sec pour s'assurer que tous les robots sont prêts à le suivre
            if(mydata->timer_init == 0){
                pogobot_timer_init(&mydata->timer, WAIT_BEFORE_WALKING);
                mydata->timer_init = 1;
            } else {
                if(pogobot_timer_has_expired(&mydata->timer)){ // timer expiré
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

int main(void) {
    pogobot_init();
#ifndef SIMULATOR
    printf("init ok\n");
#endif
    pogobot_start(user_init, user_step);
    return 0;
}