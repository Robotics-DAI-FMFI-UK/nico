// Second arduino nano with 1 mpu6050 connected and connections to both remaining arduinos

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
#include "4wcommB.h"

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
//MPU6050 mpu(0x69); // <-- use for AD0 high

MPU6050 mpu;

int16_t y3, p3, r3, y2;  // p2;

#define OUTPUT_READABLE_YAWPITCHROLL
//#define OUTPUT_READABLE_EULER

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
  
  delay(3000);

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
    //Serial.print("ypr\t");

    //int y2 = (int)(0.5 + ypr[0] * 180 / M_PI);
    //p2 = (int)(0.5 + ypr[1] * 180 / M_PI);
    //int r2 = (int)(0.5 + ypr[2] * 180 / M_PI);

    y2 = (int)(0.5 + ypr[0] * 180 / M_PI);
    
    
#endif


#ifdef OUTPUT_READABLE_EULER
    // display Euler angles in degrees
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetEuler(euler, &q);

    y2 = (int)(0.5 + euler[0] * 180 / M_PI);    
#endif


    // blink LED to indicate activity
    blinkState = !blinkState;
    digitalWrite(LED_PIN, blinkState);
    delay(20);
  }
}

//------------------------------------------------------------------

uint8_t a3_state;
uint8_t ypr16[6];

void setup() 
{
  init_4wcomm();
  init_4wcommB();
  
  a3_state = 0;
  pinMode(LED_PIN, OUTPUT);  
  Serial.begin(115200);
  mpu_setup();
  delay(1500);
  start_4wcomm();
  start_4wcommB();

  Serial.print("@");
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
    arrived_byte_from_a3(c);
  }  
}

void arrived_byte_from_a3(uint8_t c)
{
  //Serial.println(c);
  if (c == '@') a3_state = 0;
  else 
  {
    ypr16[a3_state++] = c;
    if (a3_state >= 6) 
    {
      y3 = *((int16_t *)ypr16);
      p3 = *((int16_t *)(ypr16 + 2));
      r3 = *((int16_t *)(ypr16 + 4));
      a3_state = 0;
      new_ypr3_arrived();
    }
  }
}

void new_ypr3_arrived()
{
  Serial.print("y:"); Serial.print(y3);
  Serial.print(",p:"); Serial.print(p3);
  Serial.print(",r:"); Serial.print(r3);
  //Serial.print(",d:"); Serial.println(p2-p3);
  Serial.print(",d:"); Serial.println(y2-y3);
    
    uint8_t header = '@';
    send_byte_4wcommB(header);

    for (uint8_t i = 0 ; i < 6; i++)
    send_byte_4wcommB(ypr16[i]);
    
    //int16_t to_send = p2-p3;
    int16_t to_send = y2-y3;
    send_byte_4wcommB(*((uint8_t *)&to_send));
    send_byte_4wcommB(*(((uint8_t *)&to_send) + 1));    
}
