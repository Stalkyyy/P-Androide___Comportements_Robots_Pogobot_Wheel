#include "pogobase.h"
#include <stdlib.h>
#include <time.h>

#define ID_EXTREMITY 1
#define ID_LEADER 2

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
    uint16_t type;
    uint16_t id;
} MSG;

#define MSG_SIZE sizeof(MSG) // Number of bytes

typedef union msg_template {
    uint8_t msg_array[MSG_SIZE];
    MSG msg_values;
} message;

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
    int has_leader;
    int is_leader;
    uint16_t nb_neighbours; // normalement, il devrait y avoir max 2 voisins car disposition en file indienne
    uint16_t neighbours_ids[2];
    uint16_t predecessor_id; // id du robot à suivre dans la file indienne
    int retransmit_count; // nombre de fois on retransmet le message du leader élu
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
    mydata->is_leader = 0;
    //mydata->leader_id = -1;
    mydata->has_leader = 0;
    mydata->predecessor_id = -1;
    mydata->nb_neighbours = 0;
    for (uint8_t i = 0; i < 2; i++){
        mydata->neighbours_ids[i] = UINT16_MAX;
    }
    mydata->retransmit_count=5; // on teste avec 5 pour l'instant

    pogobot_led_setColor(120, 60, 0); // jaune avant et pendant l'élection du leader
}

int id_already_received(uint16_t id){
    for (uint8_t i = 0; i < 2; i++){
        if(mydata->neighbours_ids[i] == id){
            return 1;
        }
    }
    return 0;
}

void add_id_received(uint16_t id){
    for (uint8_t i = 0; i < 2; i++){
        if(mydata->neighbours_ids[i] == UINT16_MAX){
            mydata->neighbours_ids[i] = id;
        }
    }
}

void send_id(uint16_t id){
    uint8_t msg[1];
    msg[0] = id;
    pogobot_infrared_sendLongMessage_omniSpe(msg, sizeof(msg));
}

// Définition des fonctions (elles restent inchangées)
// void send_id(uint16_t id) {
//     IDMSG msg;
//     msg.id = id;
//     pogobot_infrared_sendLongMessage_omniSpe((uint8_t *)&msg, sizeof(msg));
// }

void detect_neighbours(void) {
    send_id(mydata->my_id);

    pogobot_infrared_update();

    while (pogobot_infrared_message_available()) {
        message_t msg;
        pogobot_infrared_recover_next_message(&msg);
        // uint16_t sender_id = msg.header._sender_id;
        if (msg.payload[0] != mydata->my_id && id_already_received(msg.payload[0])==0 && mydata->nb_neighbours<=1){
        // if (sender_id != mydata->my_id && id_already_received(sender_id)==0 && mydata->nb_neighbours<=1){
            add_id_received(msg.payload[0]);
            // add_id_received(sender_id);
            mydata->nb_neighbours++;
            // printf("Robot %d a trouvé voisin %d!\n", mydata->my_id, sender_id);
            printf("Robot %d a trouvé voisin %d!\n", mydata->my_id, msg.payload[0]);
        }
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

    // printf("RETRANSIMSSION type %d\n", type);
    msg.msg_values.type = type;
    msg.msg_values.id = id;

    for ( uint16_t i = 0; i != MSG_SIZE; i++ )
        data[i] = msg.msg_array[i];
    
    if (type == ID_LEADER){ // envoi du message plusieurs fois (5 fois max à intervalle de tps régulier) car non fait une fois les rôles établis
        while (mydata->retransmit_count > 0){
            static uint32_t last_sent = 0;
            if (pogobot_stopwatch_get_elapsed_microseconds(&mydata->timer_it) - last_sent > 3000) { //après 3 sec
                printf("Temps intervalle %d", pogobot_stopwatch_get_elapsed_microseconds(&mydata->timer_it) - last_sent);
                pogobot_infrared_sendLongMessage_omniSpe((uint8_t *)(data), MSG_SIZE); 
                last_sent = pogobot_stopwatch_get_elapsed_microseconds(&mydata->timer_it);
                mydata->retransmit_count--;
            }
        }
    }
    
    pogobot_infrared_sendLongMessage_omniSpe((uint8_t *)(data), MSG_SIZE);  
}

void election_leader(void) {
    pogobot_infrared_update();
    while (pogobot_infrared_message_available()) {
        message_t msg;
        pogobot_infrared_recover_next_message(&msg);
        // IDMSG* received_msg = (IDMSG *)msg.payload;
        message received_msg;

        for (uint16_t i = 0; i != MSG_SIZE; i++)
            received_msg.msg_array[i] = msg.payload[i];

        // printf("MESSAGE extrémité reçu par %d: nbvoisins %d, myid %d VS %d\n", msg.header._sender_id, mydata->nb_neighbours, mydata->my_id, received_msg->id);
        if (received_msg.msg_values.type == ID_EXTREMITY){
            uint16_t id = received_msg.msg_values.id;
            // printf("ID RECU : %d\n", id);
            if (mydata->nb_neighbours == 1) {
                if (mydata->my_id == id){
                    break;
                }
                else if (mydata->my_id > id){
                    mydata->predecessor_id = msg.header._sender_id;
                    pogobot_led_setColor(0, 0, 255);
                    printf("PAS LEADER car: %d VS %d", mydata->my_id, id);
                    printf("Robot %d a pour prédecesseur : %d\n", mydata->my_id, mydata->predecessor_id);
                    mydata->has_leader = 1;
                }
                else if (mydata->my_id < id) {
                    mydata->is_leader = 1;
                    //mydata->leader_id = mydata->my_id;
                    pogobot_led_setColor(0, 255, 0); // vert pour le leader
                    // LEADERMSG leader_msg = { mydata->my_id };
                    printf("LEADER car myid %d < autre id %d donc envoi message leader %d\n", mydata->my_id, id, mydata->my_id);
                    // pogobot_infrared_sendLongMessage_omniSpe((uint8_t *)&leader_msg, sizeof(leader_msg));
                    transmission_msg(ID_LEADER, mydata->my_id);
                    mydata->has_leader = 1;
                }
            }
            else if (mydata->nb_neighbours > 1){
                // printf("Retransmission de l'id extremité %d\n", id);
                transmission_msg(ID_EXTREMITY, id);
                // IDMSG msg_retransmission = {received_msg->id};
                // // msg_retransmission.id = received_msg->id;
                // printf("Retransmission message avec %d = %d\n", msg_retransmission.id, received_msg->id);
                // pogobot_infrared_sendLongMessage_omniSpe((uint8_t *)&msg_retransmission, sizeof(msg_retransmission));
            }
        }
    }
}

void set_leader_and_order(void) {
    // printf("Dasns set leader and order\n");
    pogobot_infrared_update();
    while (pogobot_infrared_message_available()) {
        message_t msg;
        pogobot_infrared_recover_next_message(&msg);
        // LEADERMSG *leader_msg = (LEADERMSG *)msg.payload;
        message received_msg;

        for (uint16_t i = 0; i != MSG_SIZE; i++)
            received_msg.msg_array[i] = msg.payload[i];

        if (received_msg.msg_values.type == ID_LEADER){
            if (mydata->has_leader == 0) {
                mydata->predecessor_id = msg.header._sender_id;
                printf("SET LEADER : %d a pour prédecesseur : %d\n", mydata->my_id, mydata->predecessor_id);
                pogobot_led_setColor(0, 0, 255); // bleu pour le follower
                // pogobot_infrared_sendLongMessage_omniSpe((uint8_t *)&leader_msg, sizeof(leader_msg));
                transmission_msg(ID_LEADER, received_msg.msg_values.id);
                mydata->has_leader = 1;
            }
        }
    }
}

void send_position(uint16_t id, uint16_t ml, uint16_t mr) {
    POSITIONMSG msg = { id, ml, mr };
    pogobot_infrared_sendLongMessage_omniSpe((uint8_t *)&msg, sizeof(msg));
}

void random_walk_leader(void) {
    if (mydata->is_leader == 0) return;

    int directions[] = { 0, 1, 3, 4 };
    int random_index = rand() % 4;
    int random_direction = directions[random_index];

    uint16_t motorLeft = motorHalf;
    uint16_t motorRight = motorHalf;

    // uint16_t motorLeft;
    // uint16_t motorRight;

    switch (random_direction) {
        case 0: motorLeft += motorHalf; motorRight += motorHalf; break; // pour aller vers l'avant
        case 1: motorLeft += motorHalf; motorRight -= motorStop; break; // pour aller à droite
        case 3: motorLeft -= motorStop; motorRight += motorHalf; break; // pour aller à gauche
        case 4: motorLeft = motorStop; motorRight = motorStop; break; // pour s'arrêter
    }

    pogobot_motor_set(motorL, motorLeft);
    pogobot_motor_set(motorR, motorRight);

    send_position(mydata->my_id, motorLeft, motorRight);
}

void follow_leader(void) {
    if (mydata->is_leader == 1) return;

    pogobot_infrared_update();

    while (pogobot_infrared_message_available()) {
        message_t msg;
        pogobot_infrared_recover_next_message(&msg);
        POSITIONMSG* received_msg = (POSITIONMSG *)msg.payload;

        int direction = msg.header._receiver_ir_index;

        if (received_msg->id == mydata->predecessor_id) {
            if (direction == 0 || direction == 2){ //si predecesseur devant est détecté par en face ou derrière
                pogobot_motor_set(motorL, motorHalf);
                pogobot_motor_set(motorR, motorHalf);
                // pogobot_motor_set(motorL, received_msg->motorL);
                // pogobot_motor_set(motorR, received_msg->motorR);
            }
            else if(direction == 1){ // si prédecesseur détecté à droite
                pogobot_motor_set(motorL, motorHalf);
                pogobot_motor_set(motorR, motorQuarter);
            }
            else if(direction == 3){
                pogobot_motor_set(motorL, motorQuarter);
                pogobot_motor_set(motorR, motorHalf);
            }
            // pogobot_motor_set(motorL, received_msg->motorL);
            // pogobot_motor_set(motorR, received_msg->motorR);
            send_position(mydata->my_id, received_msg->motorL, received_msg->motorR);
        }
    }
}

void user_step(void) {
    if (mydata->has_leader == 0){
        // pogobot_timer_init(mydata->timer_it, 15e6); // 15 microsecondes
        // pogobot_timer_wait_for_expiry(mydata->timer_it);

        // if (pogobot_timer_has_expired(mydata->timer_it)){
        //printf("Robot %d : TIMER STOP\n", mydata->my_id);
        //printf("MON ID : %d\n", mydata->my_id);
        if (mydata->nb_neighbours == 0){
            detect_neighbours();
        }
        // printf("----------Robot %d : %d voisins\n", mydata->my_id, mydata->nb_neighbours);
        send_id_extremity();
        // printf("TEST APRES EXTREMITE %d\n", mydata->my_id);
        election_leader();
        // printf("TEST APRES ELECTION LEADER %d\n", mydata->my_id);
        set_leader_and_order();
        // }
    }

    // if (mydata->has_leader == 1 && mydata->retransmit_count >0){
    //     static uint32_t last_sent = 0;
    //     if (pogobot_stopwatch_elapsed_ms(&mydata->timer_it) - last_sent > 300) {
    //         transmission_msg(ID_LEADER, mydata->my_id);
    //         last_sent = pogobot_stopwatch_elapsed_ms(&mydata->timer_it);
    //     }
    //     transmission_msg(ID_LEADER, mydata->my_id); //si ce n'est pas l'id du leader ce n'est pas si grave
    //     mydata->retransmit_count--;
    // }

    // if (pogobot_stopwatch_elapsed_ms(&mydata->timer_it) - last_sent > 300) {
    //     transmission_msg(ID_LEADER, mydata->my_id);
    //     last_sent = pogobot_stopwatch_elapsed_ms(&mydata->timer_it);
    // }

    // printf("Has leader? %d\n", mydata->has_leader);
    // printf("Is leader? %d\n",  mydata->is_leader);
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