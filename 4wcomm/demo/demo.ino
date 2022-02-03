// THE SAME "CODE" GOES TO BOTH ARDUINOS, EXCEPT
// PLEASE MODIFY THE HEADER FILE 4wcomm.h FIRST LINE FOR EACH ARDUINO

#include "4wcomm.h"

void setup()
{
  Serial.begin(115200);  
  delay(1500);
  init_4wcomm();
  delay(1500);
  init_4wcomm();
  while (Serial.available()) Serial.read();
}


void loop()
{
  if (Serial.available())
  {
    char c = Serial.read();
    if (c == 'r')
    {
      Serial.println("restarting setup.");
      init_4wcomm();
    }
    else
    {
      //Serial.print("about to send "); Serial.println(c);
      send_byte_4wcomm(c);
    }
  }

  if (available_4wcomm())
  {
    uint8_t c;
    recv_byte_4wcomm(&c);    
    Serial.print((char)c);
  }
  //debug_step();
}
