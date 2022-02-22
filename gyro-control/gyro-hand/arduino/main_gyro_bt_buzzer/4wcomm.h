#ifndef __4WCOMM_H__
#define __4WCOMM_H__

//if you want to change pin connections, you will need to change interrupt setup and communication code as well

//pin connections for Arduino1 for signals A,B,C,D
#define A1_A 6
#define A1_B 5
#define A1_C 10 
#define A1_D 11


uint8_t send_byte_4wcomm(uint8_t b);

uint8_t recv_byte_4wcomm(uint8_t *b);

uint8_t available_4wcomm();

void init_4wcomm();

void start_4wcomm();

void debug_step();

#endif
