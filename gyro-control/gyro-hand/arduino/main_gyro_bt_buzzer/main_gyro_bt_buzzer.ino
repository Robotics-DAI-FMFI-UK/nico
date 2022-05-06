// first arduino nano with 1 mpu6050 connected and buzzer, BT, and connection to second arduino

// Portions of this code originate from 
// I2C device class (I2Cdev) demonstration Arduino sketch for MPU6050 class using DMP (MotionApps v6.12)
// 6/21/2012 by Jeff Rowberg <jeff@rowberg.net>
// Updates should (hopefully) always be available at https://github.com/jrowberg/i2cdevlib
/* ============================================
  I2Cdev device library code is placed under the MIT license
  Copyright (c) 2012 Jeff Rowberg

  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files (the "Software"), to deal
  in the Software without restriction, including without limitation the rights
  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
  copies of the Software, and to permit persons to whom the Software is
  furnished to do so, subject to the following conditions:

  The above copyright notice and this permission notice shall be included in
  all copies or substantial portions of the Software.

  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
  THE SOFTWARE.
  ===============================================
*/

#include "4wcomm.h"

// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps_V6_12.h"

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 mpu;
int16_t y3, p3, r3, d2, y1, p1, r1;

//MPU6050 mpu(0x69); // <-- use for AD0 high

#define OUTPUT_READABLE_YAWPITCHROLL

#define INTERRUPT_PIN 2  // use pin 2 on Arduino Uno & most boards
#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
bool blinkState = false;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 gy;         // [x, y, z]            gyro sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// packet structure for InvenSense teapot demo
uint8_t teapotPacket[14] = { '$', 0x02, 0, 0, 0, 0, 0, 0, 0, 0, 0x00, 0x00, '\r', '\n' };


volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
  mpuInterrupt = true;
}


void mpu_setup() {
  
  // join I2C bus (I2Cdev library doesn't do this automatically)
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif

  // initialize device
  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();
  pinMode(INTERRUPT_PIN, INPUT);

  // verify connection
  Serial.println(F("Testing device connections..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

  // wait for ready
  //Serial.println(F("\nSend any character to begin DMP programming and demo: "));
  //while (Serial.available() && Serial.read()); // empty buffer
  //while (!Serial.available());                 // wait for data
  //while (Serial.available() && Serial.read()); // empty buffer again

  delay(2900);
  
  // load and configure the DMP
  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();

  // supply your own gyro offsets here, scaled for min sensitivity
  mpu.setXGyroOffset(51);
  mpu.setYGyroOffset(8);
  mpu.setZGyroOffset(21);
  mpu.setXAccelOffset(1150);
  mpu.setYAccelOffset(-50);
  mpu.setZAccelOffset(1060);
  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
    // Calibration Time: generate offsets and calibrate our MPU6050
    mpu.CalibrateAccel(6);
    mpu.CalibrateGyro(6);
    Serial.println();
    mpu.PrintActiveOffsets();
    // turn on the DMP, now that it's ready
    Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);

    // enable Arduino interrupt detection
    Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
    Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
    Serial.println(F(")..."));
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();

    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    Serial.println(F("DMP ready! Waiting for first interrupt..."));
    dmpReady = true;

    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
  }
}

void mpu_loop() {
  // if programming failed, don't try to do anything
  if (!dmpReady) return;
  // read a packet from FIFO
  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) { // Get the Latest packet 

#ifdef OUTPUT_READABLE_YAWPITCHROLL
    // display Euler angles in degrees
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

    //y1 = (int)(0.5 + ypr[0] * 180 / M_PI);
    p1 = (int)(0.5 + ypr[1] * 180 / M_PI);
    r1 = (int)(0.5 + ypr[2] * 180 / M_PI);
#endif

    // blink LED to indicate activity
    blinkState = !blinkState;
    digitalWrite(LED_PIN, blinkState);
    delay(20);
  }
}


//------------------------------------------------------------------------------------------------------------
uint8_t a2_state;
uint8_t ypr16[8];

#define BUZZER 9

void setup() 
{
  init_4wcomm();
  Serial.begin(9600);
  init_serial(9600);  
  pinMode(LED_PIN, OUTPUT);  
  pinMode(BUZZER, OUTPUT);
  a2_state = 0;
  tone(BUZZER, 1760, 40);
  delay(100);
  tone(BUZZER, 880, 100);
  mpu_setup();
  delay(1500);
  start_4wcomm();
  serial_print("@");
  tone(BUZZER, 880, 40);
  delay(100);
  tone(BUZZER, 1760, 100);
  delay(100);
  for (int i = 0; i < 50; i++)
  {
    serial_print("@");
    delay(100);
    mpu_loop();
  }
  serial_print("*");
  
  tone(BUZZER, 1760, 100);
  delay(100);
  mpu_loop();
  tone(BUZZER, 1760, 100);
  delay(100);
  mpu_loop();
  tone(BUZZER, 880, 40);
  delay(100);
  mpu_loop();
  tone(BUZZER, 880, 40);
  delay(100);
}

void loop()
{
  mpu_loop();
  if (Serial.available())
  {
    char c = Serial.read();
    Serial.println(c);
    debug_step();
  }
  while (available_4wcomm())
  {
    uint8_t c;
    recv_byte_4wcomm(&c);
    arrived_byte_from_a2(c);    
  }
}

void arrived_byte_from_a2(uint8_t c)
{
  if (c == '@') a2_state = 0;
  else 
  {
    ypr16[a2_state++] = c;
    if (a2_state >= 8) 
    {
      y3 = *((int16_t *)ypr16);
      p3 = *((int16_t *)(ypr16 + 2));
      r3 = *((int16_t *)(ypr16 + 4));
      d2 = *((int16_t *)(ypr16 + 6));
      a2_state = 0;
      new_ypr32_arrived();
    }
  }
}

void new_ypr32_arrived()
{
  long chk = y3 + p3 + r3 + d2 + p1 + r1;
  serial_print(y3);
  serial_write(' ');
  serial_print(p3);
  serial_write(' ');
  serial_print(r3);
  serial_write(' ');
  serial_print(d2);
  serial_write(' ');
  //serial_print(y1);
  serial_print(p1);
  serial_write(' ');
  serial_print(r1);  
  //serial_print(p1);
  serial_write(' ');
  int ana = analogRead(0) - 271;
  chk += ana;
  serial_print(ana);
  serial_write(' ');
  ana = analogRead(1) - 271;
  chk += ana;
  serial_print(ana);
  serial_write(' ');
  ana = analogRead(2) - 537;
  chk += ana;
  serial_print(ana);
  serial_write(' ');
  ana = analogRead(3) - 271;
  chk += ana;
  serial_print(ana);
  serial_write(' ');
  serial_println(chk % 256); 
}


// the remaining part is used for serial output on pin 8 (BT), because software
// serial grabs the pin change interrupt, and thus it cannot be used

static uint16_t one_bit_write_duration;
void init_serial(uint32_t baud_rate)
{
  pinMode(7, INPUT);
  pinMode(8, OUTPUT);
  one_bit_write_duration = (uint16_t)(1000000UL / baud_rate) - 1;
}

void serial_print(int x)
{
  long n = 1;
  if (x < 0) { serial_write('-'); x *= -1; }
  while (n <= x) n *= 16;
  if (n > 1) n /= 16;    
  do
  {
    uint8_t d = x / n;
    if (d < 10) serial_write('0' + d);
    else serial_write('A' + d - 10);
    x %= n;
    n /= 16;    
  } while (n); 
}

void serial_println(int x)
{
  serial_print(x);
  serial_write(13);
  serial_write(10);
}

void serial_println(char *s)
{
  serial_print(s);  
  serial_write(13);
  serial_write(10);
}

void serial_print(char *s)
{
  while (*s) serial_write(*(s++));
}

#define ECHO_BT_TO_USB

void serial_write(uint8_t ch)
{
#ifdef ECHO_BT_TO_USB
  Serial.print((char)ch);
#endif
  PORTB &= 0b11111110;
  delayMicroseconds(one_bit_write_duration);
  for (uint8_t i = 0; i < 8; i++)
  {
    if (ch & 1) PORTB |= 0b00000001;
    else PORTB &= 0b11111110;
    ch >>= 1;
    delayMicroseconds(one_bit_write_duration);
  }
  PORTB |= 0b00000001;
  delayMicroseconds(one_bit_write_duration);
  delayMicroseconds(one_bit_write_duration);
  delayMicroseconds(one_bit_write_duration);
  delayMicroseconds(one_bit_write_duration);
  delayMicroseconds(one_bit_write_duration);
}
