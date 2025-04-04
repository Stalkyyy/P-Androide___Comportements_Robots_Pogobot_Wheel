#include "pogobase.h"
#include <stdlib.h>
#include <time.h>

// Prototypes des fonctions
void detect_neighbours(void);
void send_id_extremity(void);
void election_leader(void);
void set_leader_and_order(void);
void send_id(uint16_t id);
void send_position(uint16_t id, uint16_t ml, uint16_t mr);
void random_walk_leader(void);
void follow_leader(void);

// struct message pour partager les ids des robots (notamment ceux des robots aux extrémités de la file indienne)
typedef struct {
    uint16_t id;
} IDMSG;

// struct du message envoyé par le leader aux autres pour les prévenir du leader élu (contient id du sender = leader)
typedef struct {
    uint16_t leader_id; // contenu du msg
} LEADERMSG;

// struct du message concernant position du leader à suivre
typedef struct {
    uint16_t id; // id du sender ici
    uint16_t motorL;
    uint16_t motorR;
} POSITIONMSG;

// "Global" variables should be inserted within the USERDATA struct.
typedef struct {
    uint8_t data_foo[8];
    time_reference_t timer_it;
    uint16_t my_id;
    //uint16_t leader_id;
    bool has_leader;
    bool is_leader;
    uint16_t nb_neighbours; // normalement, il devrait y avoir max 2 voisins car disposition en file indienne
    uint16_t neighbours_ids[2];
    uint16_t predecessor_id; // id du robot à suivre dans la file indienne
} USERDATA;

DECLARE_USERDATA(USERDATA);
REGISTER_USERDATA(USERDATA);

void user_init(void) {
#ifndef SIMULATOR
    printf("setup ok\n");
#endif

    pogobot_stopwatch_reset(&mydata->timer_it);
    main_loop_hz = 60;
    max_nb_processed_msg_per_tick = 0;
    msg_rx_fn = NULL;
    msg_tx_fn = NULL;
    error_codes_led_idx = 3;

    pogobot_infrared_set_power(3);
    mydata->my_id = pogobot_helper_getid();
    mydata->is_leader = false;
    //mydata->leader_id = -1;
    mydata->has_leader = false;
    mydata->predecessor_id = -1;
    mydata->nb_neighbours = 0;
    for (uint8_t i = 0; i < 2; i++){
        mydata->neighbours_ids[i] = UINT16_MAX;
    }

    pogobot_led_setColor(120, 60, 0); // jaune avant et pendant l'élection du leader
}

bool id_already_received(uint16_t id){
    for (uint8_t i = 0; i < 2; i++){
        if(mydata->neighbours_ids[i] == id){
            return true;
        }
    }
    return false;
}

void add_id_received(uint16_t id){
    for (uint8_t i = 0; i < 2; i++){
        if(mydata->neighbours_id[i] == UINT16_MAX){
            mydata->neighbours_id[i] = id;
        }
    }
}


// Définition des fonctions (elles restent inchangées)
void send_id(uint16_t id) {
    IDMSG msg;
    msg.id = id;
    pogobot_infrared_sendLongMessage_omniSpe((uint8_t *)&msg, sizeof(msg));
}

void detect_neighbours(void) {
    send_id(mydata->my_id);

    pogobot_infrared_update();

    while (pogobot_infrared_message_available()) {
        //printf("Message trouvé !\n");
        message_t msg;
        pogobot_infrared_recover_next_message(&msg);
        IDMSG* received_msg = (IDMSG *)msg.payload;
        // printf("Robot %d a trouvé voisin !\n", mydata->my_id);
        if (received_msg->id != mydata->my_id && !id_already_received(received_msg->id)){
            mydata->nb_neighbours++;
            add_id_received(received_msg->id);
            printf("Robot %d a trouvé voisin !\n", mydata->my_id);
        }
    }
}

void send_id_extremity(void) {
    if (mydata->nb_neighbours == 1) {
        send_id(mydata->my_id);
    }
}

void election_leader(void) {
    pogobot_infrared_update();
    while (pogobot_infrared_message_available()) {
        message_t msg;
        pogobot_infrared_recover_next_message(&msg);
        IDMSG* received_msg = (IDMSG *)msg.payload;

        if (mydata->nb_neighbours == 1) {
            if (mydata->my_id < received_msg->id) {
                mydata->is_leader = true;
                //mydata->leader_id = mydata->my_id;
                mydata->has_leader = true;
                pogobot_led_setColor(0, 255, 0); // vert pour le leader
                LEADERMSG leader_msg = { mydata->my_id };
                pogobot_infrared_sendLongMessage_omniSpe((uint8_t *)&leader_msg, sizeof(leader_msg));
            }
        }
        else if (mydata->nb_neighbours > 1){
            pogobot_infrared_sendLongMessage_omniSpe((uint8_t *)&received_msg, sizeof(received_msg));
        }
    }
}

void set_leader_and_order(void) {
    pogobot_infrared_update();
    while (pogobot_infrared_message_available()) {
        message_t msg;
        pogobot_infrared_recover_next_message(&msg);
        LEADERMSG *leader_msg = (LEADERMSG *)msg.payload;

        if (!mydata->has_leader) {
            mydata->has_leader = true;
            mydata->predecessor_id = msg.header._sender_id;
            printf("Robot %d a pour prédecesseur : %d\n", mydata->my_id, mydata->predecessor_id);
            pogobot_led_setColor(0, 0, 255); // bleu pour le follower
            pogobot_infrared_sendLongMessage_omniSpe((uint8_t *)&leader_msg, sizeof(leader_msg));
        }
    }
}

void send_position(uint16_t id, uint16_t ml, uint16_t mr) {
    POSITIONMSG msg = { id, ml, mr };
    pogobot_infrared_sendLongMessage_omniSpe((uint8_t *)&msg, sizeof(msg));
}

void random_walk_leader(void) {
    if (!mydata->is_leader) return;

    int directions[] = { 0, 1, 3, 4 };
    int random_index = rand() % 4;
    int random_direction = directions[random_index];

    uint16_t motorLeft = motorThreeQuarter;
    uint16_t motorRight = motorThreeQuarter;

    switch (random_direction) {
        case 0: motorLeft += motorQuarter; motorRight += motorQuarter; break;
        case 1: motorLeft += motorQuarter; motorRight -= motorHalf; break;
        case 3: motorLeft -= motorHalf; motorRight += motorQuarter; break;
        case 4: motorLeft = motorStop; motorRight = motorStop; break;
    }

    pogobot_motor_set(motorL, motorLeft);
    pogobot_motor_set(motorR, motorRight);

    send_position(mydata->my_id, motorLeft, motorRight);
}

void follow_leader(void) {
    if (mydata->is_leader) return;

    pogobot_infrared_update();

    while (pogobot_infrared_message_available()) {
        message_t msg;
        pogobot_infrared_recover_next_message(&msg);
        POSITIONMSG* received_msg = (POSITIONMSG *)msg.payload;

        if (received_msg->id == mydata->predecessor_id) {
            pogobot_motor_set(motorL, received_msg->motorL);
            pogobot_motor_set(motorR, received_msg->motorR);
            send_position(mydata->my_id, received_msg->motorL, received_msg->motorR);
        }
    }
}

void user_step(void) {
    if (!mydata->has_leader){
        // pogobot_timer_init(mydata->timer_it, 15e6); // 15 microsecondes
        // pogobot_timer_wait_for_expiry(mydata->timer_it);

        // if (pogobot_timer_has_expired(mydata->timer_it)){
        //printf("Robot %d : TIMER STOP\n", mydata->my_id);
        detect_neighbours();
        printf("----------Robot %d : %d voisins\n", mydata->my_id, mydata->nb_neighbours);
        send_id_extremity();
        election_leader();
        set_leader_and_order();
        // }
    }
    
    //printf("leader : %d\n", mydata->leader_id);
    // if(!mydata->has_leader){
    //     // printf("Dans la boucle\n");
    //     detect_neighbours();
    //     send_id_extremity();
    //     election_leader();
    //     set_leader_and_order();
    // }

    random_walk_leader();
    follow_leader();
}

int main(void) {
    pogobot_init();
#ifndef SIMULATOR
    printf("init ok\n");
#endif
    pogobot_start(user_init, user_step);
    return 0;
}