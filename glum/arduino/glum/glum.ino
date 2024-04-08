// servo ranges:
//
// LEFT TOP LID (0)t
// 
//   132 completely closed
//    33 maximally open  (dependent on VERTICAL!!)
//    
// RIGHT TOP LID (1)
// 
//   145 compeltely open (dependent on VERTICAL)
//    40 maximally closed   
//    
// BOTTOM RIGHT LID (2)
// 
//    164  lid is in the nose
//     26  maximally closed
//     
// BOTTOM LEFT LID (3)
// 
//      5  extremely open down
//    164  maximally closed
//    
// VERTICAL
//     93  LOOKING DOWN
//     161 LOOKING UP
//     
// HORIZONTAL
//     29  COMPLETELY RIGHT
//    138  COMPLETELY LEFT

#include <Servo.h>

#define SERVO_DOF             6

#define TOP_LEFT_LID           0
#define TOP_RIGHT_LID         1
#define BOTTOM_LEFT_LID   2
#define BOTTOM_RIGHT_LID 3
#define VERTICAL                  4
#define HORIZONTAL            5

#define PIN_TOP_LEFT_LID           4
#define PIN_TOP_RIGHT_LID         6
#define PIN_BOTTOM_LEFT_LID   5
#define PIN_BOTTOM_RIGHT_LID 7
#define PIN_VERTICAL                  3
#define PIN_HORIZONTAL            2

#define PIN_BUZZER                     8

Servo  servos[SERVO_DOF];

int current_servo_position[SERVO_DOF];
int wished_servo_position[SERVO_DOF];
int servo_speed[SERVO_DOF];

unsigned long time_start;

void setup()
{ 
   servos[TOP_LEFT_LID].attach(PIN_TOP_LEFT_LID);
   servos[TOP_RIGHT_LID].attach(PIN_TOP_RIGHT_LID);
   servos[BOTTOM_LEFT_LID].attach(PIN_BOTTOM_LEFT_LID);
   servos[BOTTOM_RIGHT_LID].attach(PIN_BOTTOM_RIGHT_LID);
   servos[VERTICAL].attach(PIN_VERTICAL);
   servos[HORIZONTAL].attach(PIN_HORIZONTAL);
   pinMode(PIN_BUZZER, OUTPUT);
   tone(PIN_BUZZER, 1760, 300);
   delay(330);
   tone(PIN_BUZZER, 880, 100);
   delay(130);
   tone(PIN_BUZZER, 880, 200);
   
   for (int i = 0; i < SERVO_DOF; i++)
   {
     servos[i].write(90);
     current_servo_position[i] = 90;
     wished_servo_position[i] = 90;
     servo_speed[i] = 1;
   }
   Serial.begin(115200);
   Serial.println("Glum is glooming");
   time_start = millis();
}

void print_servo(int s)
{
    Serial.print(s);
    Serial.print(": ");
    Serial.println(current_servo_position[s]);
}

void increment_servo(int s)
{
  if (current_servo_position[s] < 180) 
  {
    current_servo_position[s]++;
    servos[s].write(current_servo_position[s]);
    print_servo(s);
  }
}

void decrement_servo(int s)
{
  if (current_servo_position[s] > 0) 
  {
    current_servo_position[s]--;
    servos[s].write(current_servo_position[s]);
    print_servo(s);
  }
}

const char *control_keys = "azsxdcfvgbhn";

void test_mode()
{
   char c = Serial.read();
   while ((c != 27) && (c != 'Q'))
   {
     for (int i = 0; i < SERVO_DOF * 2; i++)
     {
         if (c == control_keys[i])
         {
           if (i & 1) decrement_servo(i >> 1);
           else increment_servo(i >> 1);
         }
     }
     c = Serial.read();
   }
}

void one_servo_step(int i)
{
  int servo_delta = (wished_servo_position[i] > current_servo_position[i]) ?servo_speed[i]:-servo_speed[i];
  if (abs(wished_servo_position[i] - current_servo_position[i]) < servo_speed[i])
      current_servo_position[i] = wished_servo_position[i];
  else 
  {
    current_servo_position[i] += servo_delta;
    servos[i].write(current_servo_position[i]);
  }
}

void servo_step()
{
  for (int i = 0; i < SERVO_DOF; i++)
  {
     if (wished_servo_position[i] != current_servo_position[i])
        one_servo_step(i);
  }
}

#define BUF_SIZE 30
char buf[BUF_SIZE];
int buf_wp;

void read_line()
{
  buf_wp = 0;
  char c = Serial.read();
  while (c != '\n')
  {
    buf[buf_wp++] = c;
    if (buf_wp == BUF_SIZE - 1)
    {
      buf[buf_wp] = 0;
      break;
    }
    c = Serial.read();
  }
}

char *read_number_from_buffer(int *value, char *bp)
{
  *value = 0;
  while (*bp && (*bp >= '0') && (*bp <= '9'))
  {
    *value *= 10;
    *value += *bp - '0';
    bp++;
  }
  bp++;
  return bp;
}

void read_speed_command()
{
   Serial.read();  // skip space
   read_line();  
   int which_servo, sspeed;
   char *bp = read_number_from_buffer(&which_servo, buf);
   bp = read_number_from_buffer(&sspeed, bp);
   if ((which_servo < 0) || (which_servo > SERVO_DOF) || (sspeed < 1) || (sspeed > 20))
   {
     Serial.print("! incorrect speed command ");
     Serial.print(which_servo);
     Serial.print(":");
     Serial.println(sspeed);
     while (Serial.available()) Serial.read();
   }
   else 
   {
    servo_speed[which_servo] = sspeed;
    Serial.print("speed ");
    Serial.print(which_servo);
    Serial.print(" ");
    Serial.println(sspeed);
   }
}

void read_target_command()
{
   Serial.read();  // skip space
   read_line();   
   char *bp = buf;
   Serial.print("pos ");
   for (int i = 0; i < 6; i++)
   {
     int servo_target;
     bp = read_number_from_buffer(&servo_target, bp);
     if ((servo_target < 0) || (servo_target > 180))
     {
        Serial.println("! incorrect servo target command");
        while (Serial.available()) Serial.read();
        break;
     }
     else
     {
       wished_servo_position[i] = servo_target;
       Serial.print(i);
       Serial.print(": ");
       Serial.print(servo_target);
       Serial.print("|");
     }
   }
   Serial.println();
}

#define DEMO_STEPS 6
int demo[DEMO_STEPS][6] = { {45, 130, 90, 90, 90, 90 }, 
                            {45, 130, 90, 90, 90, 30 }, 
                            {45, 130, 90, 90, 90, 120 },
                            {45, 130, 90, 90, 90, 90 }, 
                            { 130, 40, 30, 130, 90, 90 }, 
                            {40, 140, 120, 60, 90, 90} };

int demo_time[6] = { 3000, 6000, 9000, 12000, 13000, 16000 };                            

int demo_speed[6] = { 1, 1, 1, 1, 15, 1 };                      

int current_speed = 1;
int demo_step = 0;


void dodemo(int d)
{
  for (int i = 0; i < 6; i++)
    wished_servo_position[i] = demo[d][i];
}
                           

void loop()
{
  /*
  if (Serial.available())
  {
    char c = Serial.read();
    if (c == ' ')
       test_mode();
    else if (c == 's')
       read_speed_command();
    else if (c == 't')
       read_target_command();
  }*/

  if (millis() - time_start > demo_time[demo_step])
  {
    if (demo_speed[demo_step] != current_speed)
    {
      for (int i = 0; i < 6; i++)
        servo_speed[i] = demo_speed[demo_step];
    }
    dodemo(demo_step);
    demo_step++;
    if (demo_step == DEMO_STEPS)
    {
      demo_step = 0;
      time_start = millis();
    }    
  }
  
  servo_step();
  delay(10);
}
