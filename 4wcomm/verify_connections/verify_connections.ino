#include "4wcomm.h"

// download with the following line to Arduino1, and without it to Arduino2
// if connections are physically ok, the programs will advance in both

//#define THIS_IS_ARDUINO1

void setup() 
{
#ifdef THIS_IS_ARDUINO1
  pinMode(A1_A, OUTPUT);
  pinMode(A1_C, OUTPUT);
  pinMode(A1_B, INPUT);
  pinMode(A1_D, INPUT);
  digitalWrite(A1_A, HIGH);
  digitalWrite(A1_C, HIGH);
#else
  pinMode(A2_A, INPUT);
  pinMode(A2_C, INPUT);
  pinMode(A2_B, OUTPUT);
  pinMode(A2_D, OUTPUT);
  digitalWrite(A2_B, HIGH);
  digitalWrite(A2_D, HIGH);
#endif
  Serial.begin(115200);
  Serial.println("starting in 3 sec...");
  delay(3000);
}

void loop() 
{
#ifdef THIS_IS_ARDUINO1
  loop1();
#else
  loop2();
#endif
}

void loop1()
{
  Serial.println("A = 0");
  delay(1000);
  digitalWrite(A1_A, LOW);
  Serial.println("waiting for B=0");
  while (digitalRead(A1_B));
  digitalWrite(A1_A, HIGH);
  Serial.println("B is now 0, A = 1; C = 0");
  delay(1000);
  digitalWrite(A1_C, LOW);
  Serial.println("waiting for B=1 and then for D=0");
  while (!digitalRead(A1_B));
  Serial.println("B is now 1");
  while (digitalRead(A1_D));
  delay(1000);
  Serial.println("D is now 0, C = 1");
  delay(1000);
  digitalWrite(A1_C, HIGH);
}

void loop2()
{  
  Serial.println("waiting for A=0");
  while (digitalRead(A2_A));
  Serial.println("A is now 0, set B = 0");
  delay(1000);
  digitalWrite(A2_B, LOW);
  Serial.println("waiting for A=1");
  while (!digitalRead(A2_A));  
  Serial.println("A is now 1, setting B=1, waiting for C=0");
  delay(1000);
  digitalWrite(A2_B, HIGH);
  while (digitalRead(A2_C));
  Serial.println("C is now 0, setting D = 0, waiting for C = 1");
  delay(1000);
  digitalWrite(A2_D, LOW);
  while (!digitalRead(A2_C));
  Serial.println("C is now 1, setting D=1");
  digitalWrite(A2_D, HIGH);
  delay(1000);
}
