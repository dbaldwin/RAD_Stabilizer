#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"
#include "Servo.h"

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 mpu;

#define INTERRUPT_PIN 2  // use pin 2 on Arduino Uno & most boards

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
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// Servos connected to elevons
Servo ch2Servo; 
Servo ch3Servo;

byte last_ch1, last_ch2, last_ch3, last_ch5; // throttle, aileron, elevator, auxiliary
volatile int receiver_ch_1, receiver_ch_2, receiver_ch_3, receiver_ch_5; // store values for each channel
unsigned long ch1_timer, ch2_timer, ch3_timer, ch5_timer, current_time;
volatile bool stabilize_on = false;

// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
  mpuInterrupt = true;
}



// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup() {


  // Setup interrupts for Arduino Micro
  // Refer to this for interrupt pins:
  // https://www.theengineeringprojects.com/wp-content/uploads/2018/09/introduction-to-Arduino-Micro-3-3.png
  PCICR |= (1 << PCIE0);    // set PCIE0 to enable PCMSK0 scan
  PCMSK0 |= (1 << PCINT0);  // set PCINT0 (digital input 8 - ch1 throttle)
  PCMSK0 |= (1 << PCINT1);  // set PCINT1 (digital input 9 - ch2 left elevon)
  PCMSK0 |= (1 << PCINT2);  // set PCINT2 (digital input 10 - ch3 right elevon)
  PCMSK0 |= (1 << PCINT3);  // set PCINT3 (digital input 11 - ch5 auxiliary switch)

  Wire.begin();
  Wire.setClock(400000);

  // initialize serial communication
  // (115200 chosen because it is required for Teapot Demo output, but it's
  // really up to you depending on your project)
  Serial.begin(115200);
  while (!Serial); // wait for Leonardo enumeration, others continue immediately

  // NOTE: 8MHz or slower host processors, like the Teensy @ 3.3V or Arduino
  // Pro Mini running at 3.3V, cannot handle this baud rate reliably due to
  // the baud timing being too misaligned with processor ticks. You must use
  // 38400 or slower in these cases, or use some kind of external separate
  // crystal solution for the UART timer.

  // initialize device
  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();
  pinMode(INTERRUPT_PIN, INPUT);

  // verify connection
  Serial.println(F("Testing device connections..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

  // load and configure the DMP
  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();

  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
    // Calibration Time: generate offsets and calibrate our MPU6050
    mpu.CalibrateAccel(6);
    mpu.CalibrateGyro(6);
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

  // Attach left servo to D5 (PWM pin)
  ch2Servo.attach(5); 

  // Attach right servo to D6 (PWM pin);
  ch3Servo.attach(6);

  // This should be center position
  ch2Servo.write(90);
  ch3Servo.write(90);

}



// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop() {
  
  // if programming failed, don't try to do anything
  if (!dmpReady) return;

  // wait for MPU interrupt or extra packet(s) available
  while (!mpuInterrupt && fifoCount < packetSize) {
    if (mpuInterrupt && fifoCount < packetSize) {
      // try to get out of the infinite loop
      fifoCount = mpu.getFIFOCount();
    }
  }

  // reset interrupt flag and get INT_STATUS byte
  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();

  // get current FIFO count
  fifoCount = mpu.getFIFOCount();
  if (fifoCount < packetSize) {
    //Lets go back and wait for another interrupt. We shouldn't be here, we got an interrupt from another event
    // This is blocking so don't do it   while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
  }
  // check for overflow (this should never happen unless our code is too inefficient)
  else if ((mpuIntStatus & (0x01 << MPU6050_INTERRUPT_FIFO_OFLOW_BIT)) || fifoCount >= 1024) {
    // reset so we can continue cleanly
    mpu.resetFIFO();
    //  fifoCount = mpu.getFIFOCount();  // will be zero after reset no need to ask
    Serial.println(F("FIFO overflow!"));

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
  } else if (mpuIntStatus & (0x01 << MPU6050_INTERRUPT_DMP_INT_BIT)) {

    // read a packet from FIFO
    while (fifoCount >= packetSize) { // Lets catch up to NOW, someone is using the dreaded delay()!
      mpu.getFIFOBytes(fifoBuffer, packetSize);
      // track FIFO count here in case there is > 1 packet available
      // (this lets us immediately read more without waiting for an interrupt)
      fifoCount -= packetSize;
    }

    // Get Euler angles in degrees
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);


    // Uncomment these for bench testing so we don't have to worry about aux input
//    receiver_ch_5 = 1501;
//    receiver_ch_2 = 1500;
//    receiver_ch_3 = 1500;

    // We're in stablize mode because the auxiliary channel is high
    if(receiver_ch_5 > 1500) {

      // Manual input is being given
      if(receiver_ch_2 < 1480 || receiver_ch_2 > 1520 || receiver_ch_3 < 1480 || receiver_ch_3 > 1520) {
        
        applyManualInput();

      // Stabilize
      } else {

        // Let's use the gyro to stabilize the servos
        float roll_angle = ypr[1] * 180 / M_PI * -1; // Radians to degrees
        float pitch_angle = ypr[2] * 180 / M_PI; // Radians to degrees

        // Convert gyro angles to servo outputs
        float roll_signal = map(roll_angle, -90, 90, 0, 180);
        float pitch_signal = map(pitch_angle, -90, 90, 0, 180);

        // Mix for right elevon
        float ch3_mix = abs((roll_signal + pitch_signal) * 0.5);
  
        if (pitch_angle > 0) {
          pitch_signal = map(pitch_angle, 90, -90, 0, 180);
        } else if (pitch_angle < 0) {
          pitch_signal = map(pitch_angle, -90, 90, 180, 0);
        }

        // Mix for left elevon
        float ch2_mix = abs((roll_signal + pitch_signal) * 0.5);
  
//        Serial.print("Pitch: ");
//        Serial.print(pitch_signal);
//        Serial.print("\t Roll: ");
//        Serial.print(roll_signal);
//        Serial.print("\t Ch2 mix: ");
//        Serial.print(ch2_mix);
//        Serial.print("\t Ch3 mix: ");
//        Serial.println(ch3_mix);

        // Write the stabilized values to the servos
        ch2Servo.write(ch2_mix);
        ch3Servo.write(ch3_mix);
      }

    // We're in manual mode so let the values pass through to the servos
    } else {

      applyManualInput();
      
    }
    
  }
}

void applyManualInput() {
  Serial.print(receiver_ch_2);
  Serial.print("\t");
  Serial.println(receiver_ch_3);
  
  float invert_ch_2 = map(receiver_ch_2, 1000, 2000, 2000, 1000);
  // Write these values to the servo
  ch2Servo.writeMicroseconds(invert_ch_2);
  ch3Servo.writeMicroseconds(receiver_ch_3);
}

// For port numbers with Arduino Nano:
// https://components101.com/sites/default/files/component_pin/Arduino-Nano-Pinout.png
ISR(PCINT0_vect) {

  current_time = micros();

  // CH1 (throttle) = digital pin 8 (PB0)
  if(PINB & B00000001) {                                 // Pin 8 (PB0) is high
    if(last_ch1 == 0){                                   // Input 8 went high
      last_ch1 = 1;                                      // Store current state
      ch1_timer = current_time;                          // Set the timer
    }
  } else if(last_ch1 == 1){                              // Input 8 is not high and changed from 1 to 0
    last_ch1 = 0;                                        // Remember current input state
    receiver_ch_1 = current_time - ch1_timer;            // Channel 1 is current_time - timer_1
  }

  // CH2 (left elevon) = digital pin 9 (PB1)
  if(PINB & B00000010) {                                 // Pin 9 (PB1) is high
    if(last_ch2 == 0){                                   // Input 9 changed from 0 to 1
      last_ch2 = 1;                                      // Remember current input state
      ch2_timer = current_time;                          // Set timer_2 to current_time
    }
  } else if(last_ch2 == 1){                              // Input 9 is not high and changed from 1 to 0
    last_ch2 = 0;                                        // Remember current input state
    receiver_ch_2 = current_time - ch2_timer;            // Channel 2 is current_time - timer_2
  }

  // CH3 (right elevon) = digital pin 10 (PB2)
  if(PINB & B00000100) {                                 // Pin 10 (PB2) is high
    if(last_ch3 == 0){                                   // Input 10 changed from 0 to 1
      last_ch3 = 1;                                      // Remember current input state
      ch3_timer = current_time;                          // Set timer_3 to current_time
    }
  } else if(last_ch3 == 1){                              // Input 10 is not high and changed from 1 to 0
    last_ch3 = 0;                                        // Remember current input state
    receiver_ch_3 = current_time - ch3_timer;            // Channel 3 is current_time - timer_3
  }

  // CH5 (auxiliary) = digital pin 11 (PB3)
  if(PINB & B00001000) {                                 // Pin 11 (PB3) is high
    if(last_ch5 == 0){                                   // Input 11 changed from 0 to 1
      last_ch5 = 1;                                      // Remember current input state
      ch5_timer = current_time;                          // Set timer_5 to current_time
    }
  } else if(last_ch5 == 1){                              // Input 11 is not high and changed from 1 to 0
    last_ch5 = 0;                                        // Remember current input state
    receiver_ch_5 = current_time - ch5_timer;            // Channel 5 is current_time - timer_5
  }
  
  
}
