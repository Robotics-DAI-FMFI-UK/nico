#ifndef __4WCOMM_H__
#define __4WCOMM_H__

// LEAVE THIS HERE FOR FIRST ARDUINO AND COMMENT IT OUT FOR THE SECOND ARDUINO
#define THIS_IS_ARDUINO1

//if you want to change pin connections, you will need to change interrupt setup and communication code as well

//pin connections for Arduino1 for signals A,B,C,D
#define A1_A 6
#define A1_B 5
#define A1_C 7
#define A1_D 8

//pin connections for Arduino2 for signals A,B,C,D
#define A2_A 3
#define A2_B 4
#define A2_C 7
#define A2_D 8



uint8_t send_byte_4wcomm(uint8_t b);

uint8_t recv_byte_4wcomm(uint8_t *b);

uint8_t available_4wcomm();

void init_4wcomm();

void debug_step();

#endif
