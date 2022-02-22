#ifndef __4WCOMMB_H__
#define __4WCOMMB_H__

//if you want to change pin connections, you will need to change interrupt setup and communication code as well

//pin connections for Arduino2 for signals A,B,C,D
#define B_A2_A 3
#define B_A2_B 4
#define B_A2_C 9
#define B_A2_D 10

uint8_t send_byte_4wcommB(uint8_t b);

uint8_t recv_byte_4wcommB(uint8_t *b);

uint8_t available_4wcommB();

void init_4wcommB();

void start_4wcommB();

void debug_stepB();

#endif
