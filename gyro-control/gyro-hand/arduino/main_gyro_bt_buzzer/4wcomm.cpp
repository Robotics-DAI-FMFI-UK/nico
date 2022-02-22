#include <Arduino.h>

#include "4wcomm.h"

#define STATE_4WCOMM_IDLE 1
#define STATE_4WCOMM_SEND_REQUEST_SENT 2
#define STATE_4WCOMM_SEND_REQUEST_CONFIRMED 3
#define STATE_4WCOMM_SEND_REQUEST_CONF_RECEIVED 4
#define STATE_4WCOMM_GETTING_READY_FOR_DATA 5
#define STATE_4WCOMM_DATA_TRANSMISSION 6
#define STATE_4WCOMM_DATA_TRANS_CONFIRMED 7
#define STATE_4WCOMM_DATA_TRANS_CONF_RECEIVED 8

#define STATE_4WCOMM_INIT1 9
#define STATE_4WCOMM_INIT2 10
#define STATE_4WCOMM_INIT3 11
#define STATE_4WCOMM_INIT4 12
#define STATE_4WCOMM_INIT5 13
#define STATE_4WCOMM_INIT6 14
#define STATE_4WCOMM_INIT7 15
#define STATE_4WCOMM_INIT8 16

#define DIR_4WCOMM_A1_TO_A2 1
#define DIR_4WCOMM_A2_TO_A1 2
#define DIR_4WCOMM_NONE     3

#define RECVBUF_SIZE 50
#define SENDBUF_SIZE 50

static volatile uint8_t comm_state;
static volatile uint8_t comm_dir;

static uint8_t sendbuf[SENDBUF_SIZE];
static volatile uint8_t sendbuf_readptr;
static volatile uint8_t sendbuf_writeptr;
static volatile uint8_t sending_byte;
static volatile uint8_t sending_bit_number;

static uint8_t recvbuf[RECVBUF_SIZE];
static volatile uint8_t recvbuf_readptr;
static volatile uint8_t recvbuf_writeptr;
static volatile uint8_t recving_byte;
static volatile uint8_t recving_bit_number;

static void initiate_sending();
static void new_bit_received(uint8_t b);
static uint8_t next_bit_to_send();

static volatile uint32_t int0_count = 0;
static volatile uint32_t int2_count = 0;
static volatile uint32_t dbg1 = 0;

ISR(PCINT0_vect)
{
  int0_count++;
  
  uint8_t D_raising = PINB & 8;
  
  if (D_raising) 
  {
      if (comm_state == STATE_4WCOMM_DATA_TRANS_CONFIRMED) 
      {
          comm_state = STATE_4WCOMM_IDLE;
          comm_dir = DIR_4WCOMM_NONE;
          PORTB |= 0b00000100; // raise C
          if ((sending_bit_number != 7) || (sendbuf_readptr != 255)) //more to send?
             initiate_sending();
      }
      else if (comm_state == STATE_4WCOMM_INIT8)
      {
          comm_state = STATE_4WCOMM_IDLE;
      }
      else if (comm_dir == DIR_4WCOMM_A2_TO_A1)
      {
          comm_state = STATE_4WCOMM_GETTING_READY_FOR_DATA;
          PORTB |= 0b00000100; // raise C
      }
      
  }
  else //D falling
  {
      if (comm_state == STATE_4WCOMM_INIT6)
      {
          comm_state = STATE_4WCOMM_INIT8;
          PORTB |= 0b00000100; // raise C
      }
      else if (comm_dir != DIR_4WCOMM_A1_TO_A2)  // A1_TO_A2 has a higher priority
      {
          comm_dir = DIR_4WCOMM_A2_TO_A1;
          if (comm_state == STATE_4WCOMM_GETTING_READY_FOR_DATA) 
              new_bit_received((PIND >> 5) & 1);
          comm_state += 2;
          PORTB &= 0b11111011; // pulldown C
      }
  }
}

ISR(PCINT2_vect)
{
  int2_count++;
  
  uint8_t B_raising = PIND & 32; 
  if (comm_dir == DIR_4WCOMM_A1_TO_A2)
  {
      if (B_raising)
      {
          if (comm_state == STATE_4WCOMM_SEND_REQUEST_CONF_RECEIVED)
          {              
              uint8_t old_portb = PORTB;
              old_portb &= 0b11111011; // send DATA on C
              old_portb |= (next_bit_to_send() << 2);
              PORTB = old_portb;
              PORTD &= 0b10111111;  // pulldown A
              comm_state += 2;
          }
          else 
          {
              comm_state = STATE_4WCOMM_IDLE;
              comm_dir = DIR_4WCOMM_NONE;
              if ((sending_bit_number != 7) || (sendbuf_readptr != 255)) //more to send?
                 initiate_sending();
          }
      }
      else // B falling
      {
          if (comm_state == STATE_4WCOMM_DATA_TRANSMISSION)
          {
              comm_state = STATE_4WCOMM_DATA_TRANS_CONF_RECEIVED;
              PORTB |= 0b00000100; // clear data - raise C
          }
          else 
          {
              comm_state = STATE_4WCOMM_SEND_REQUEST_CONF_RECEIVED;
          }
          PORTD |= 0b01000000;  // raise A
      }
  }
  else
  {
      if (B_raising && (comm_state == STATE_4WCOMM_INIT4))
      {
          comm_state = STATE_4WCOMM_INIT6;
          PORTB &= 0b11111011; // pulldown C
      }
      else if ((!B_raising) && (comm_state == STATE_4WCOMM_INIT2))
      {
          comm_state = STATE_4WCOMM_INIT4;
          PORTD |= 0b01000000;  // raise A
      }
  }
}


void init_send_recv_bufs()
{
    sendbuf_writeptr = 0;
    sendbuf_readptr = 255;  // empty
    recvbuf_writeptr = 0;
    recvbuf_readptr = 255;  // empty
    recving_byte = 0;
    sending_byte = 0;
    sending_bit_number = 7;
    recving_bit_number = 7;
}

static uint8_t next_bit_to_send()
{
    if (sending_bit_number == 7) // next byte
    {
       if (sendbuf_readptr == 255) // empty? (should never happen)
           return 0;
       sending_bit_number = 0;
       sending_byte = sendbuf[sendbuf_readptr++];
       if (sendbuf_readptr == SENDBUF_SIZE) sendbuf_readptr = 0;
       if (sendbuf_readptr == sendbuf_writeptr) sendbuf_readptr = 255; //became empty
    }
    else sending_bit_number++;
    return (sending_byte >> sending_bit_number) & 1; 
}

static void initiate_sending()
{
   PORTD &= 0b10111111;  // pulldown A 
   comm_dir = DIR_4WCOMM_A1_TO_A2;
   comm_state = STATE_4WCOMM_SEND_REQUEST_SENT;
}

static void new_bit_received(uint8_t b)
{
    if (recving_bit_number == 7)  // next byte
    {
       recving_bit_number = 0;
    }
    else recving_bit_number++;
    
    recving_byte |= (b << recving_bit_number);    
    if (recving_bit_number == 7)
    {
       if (recvbuf_writeptr == recvbuf_readptr) return;  // full? => ignore
       recvbuf[recvbuf_writeptr] = recving_byte;
       if (recvbuf_readptr == 255) recvbuf_readptr = recvbuf_writeptr;
       recvbuf_writeptr++;
       if (recvbuf_writeptr == RECVBUF_SIZE) recvbuf_writeptr = 0;
       recving_byte = 0;
    }
}

uint8_t send_byte_4wcomm(uint8_t b)
{
    while (comm_state >= STATE_4WCOMM_INIT1) // wait if hand-shake incomplete
    {
      Serial.println(comm_state);
      delay(500);
    }
    
    cli();
    if (sendbuf_writeptr == sendbuf_readptr) { sei(); return 0; } //full
    sendbuf[sendbuf_writeptr] = b;
    if (sendbuf_readptr == 255) 
    {
        sendbuf_readptr = sendbuf_writeptr;
        if ((sending_bit_number == 7) && (comm_state == STATE_4WCOMM_IDLE)) initiate_sending();
    }
    sendbuf_writeptr++;
    if (sendbuf_writeptr == SENDBUF_SIZE) sendbuf_writeptr = 0;
    sei();
    return 1;
}

uint8_t recv_byte_4wcomm(uint8_t *b)
{
   cli();
   if (recvbuf_readptr == 255) { sei(); return 0; } //empty
   *b = recvbuf[recvbuf_readptr];
   recvbuf_readptr++;
   if (recvbuf_readptr == RECVBUF_SIZE) recvbuf_readptr = 0;
   if (recvbuf_readptr == recvbuf_writeptr) recvbuf_readptr = 255;  //became empty
   sei();
   return 1; 
}

uint8_t available_4wcomm()
{
   //Serial.print("available: "); Serial.println(recvbuf_readptr);
   return recvbuf_readptr != 255;  
}

void init_4wcomm()
{
  init_send_recv_bufs();
  comm_state = STATE_4WCOMM_INIT2;
  comm_dir = DIR_4WCOMM_NONE;
  pinMode(A1_A, OUTPUT);
  pinMode(A1_B, INPUT);
  pinMode(A1_C, OUTPUT);
  pinMode(A1_D, INPUT);
  digitalWrite(A1_A, HIGH);
  digitalWrite(A1_C, HIGH);
  //pullups on
  digitalWrite(A1_B, HIGH);
  digitalWrite(A1_D, HIGH);
}

void start_4wcomm()
{  
  PCIFR = 0;
  PCMSK0 |= 0b1000;    //PCINT3
  PCMSK2 |= 0b00100000; //PCINT21
  PCICR |= 0b101;  //PCIE0 + PCIE2
  while (((PIND & 32) && (PINB & 8)) == 0); // wait for start of hand-shake
  while (comm_state == STATE_4WCOMM_INIT2)
  {
    PORTD &= 0b10111111;  // pulldown A, start hand-shake
    delayMicroseconds(5);    
    PORTD |= 0b01000000;  // raise A, 
    delayMicroseconds(5);
  }  
}

void debug_step()
{
  static  uint8_t last_s = 0;
  static  uint32_t last_0 = 0;
  static  uint32_t last_2 = 0;

  if ((last_s != comm_state) || (last_0 != int0_count) || (last_2 != int2_count))
  {
    Serial.print(millis()); Serial.print(": ");
    Serial.print("s:"); Serial.print(comm_state);
    Serial.print(", i0:"); Serial.print(int0_count);
    Serial.print(", i2:"); Serial.print(int2_count);
    Serial.print(", d:"); Serial.print(dbg1);
    Serial.print(", pid:"); Serial.print(PIND);
    Serial.print(", pod:"); Serial.print(PORTD);
    Serial.print(", pob:"); Serial.print(PORTB);
    Serial.print(", rb:"); Serial.print(recving_byte);
    Serial.print(", rbn:"); Serial.print(recving_bit_number);
    Serial.print(", rbrp:"); Serial.print(recvbuf_readptr);
    Serial.print(", rbwp:"); Serial.println(recvbuf_writeptr);
    last_s = comm_state;
    last_0 = int0_count;
    last_2 = int2_count;
  }
}
