// Currently specific to Arduino Micro

byte last_ch1, last_ch2, last_ch3;
volatile int receiver_ch_1, receiver_ch_2, receiver_ch_3;
unsigned long ch1_timer, ch2_timer, ch3_timer, current_time;

void setup() {

  // Setup interrupts for Arduino Micro
  PCICR |= (1 << PCIE0);    // set PCIE0 to enable PCMSK0 scan
  PCMSK0 |= (1 << PCINT4);  // set PCINT4 (digital input 8)
  PCMSK0 |= (1 << PCINT5);  // set PCINT5 (digital input 9)
  PCMSK0 |= (1 << PCINT6);  // set PCINT2 (digital input 10)
  
}

void loop() {
  
  Serial.print("Throttle: ");
  Serial.print(receiver_ch_1);
  Serial.print("\tCH2: ");
  Serial.print(receiver_ch_2);
  Serial.print("\tCH3: ");
  Serial.println(receiver_ch_3);
  
}

// For port numbers with Arduino Micro refer to this:
// https://content.arduino.cc/assets/Pinout-Micro_latest.png
ISR(PCINT0_vect) {

  current_time = micros();

  // Throttle = digital pin 8 (PB4)
  if(PINB & B00010000) {                                   // Pin 8 (PB4) is high
    if(last_ch1 == 0){                                   // Input 8 went high
      last_ch1 = 1;                                      // Store current state
      ch1_timer = current_time;                              // Set the timer
    }
  } else if(last_ch1 == 1){                              //Input 8 is not high and changed from 1 to 0
    last_ch1 = 0;                                        //Remember current input state
    receiver_ch_1 = current_time - ch1_timer;         //Channel 1 is current_time - timer_1
  }

  // Left elevon = digital pin 9 (PB5)
  if(PINB & B00100000) {                                       //Is input 9 high?
    if(last_ch2 == 0){                                   //Input 9 changed from 0 to 1
      last_ch2 = 1;                                      //Remember current input state
      ch2_timer = current_time;                                  //Set timer_2 to current_time
    }
  } else if(last_ch2 == 1){                              //Input 9 is not high and changed from 1 to 0
    last_ch2 = 0;                                        //Remember current input state
    receiver_ch_2 = current_time - ch2_timer;         //Channel 2 is current_time - timer_2
  }

  // Right elevon = digital pin 10 (PB6)
  if(PINB & B01000000) {                                       //Is input 10 high?
    if(last_ch3 == 0){                                   //Input 10 changed from 0 to 1
      last_ch3 = 1;                                      //Remember current input state
      ch3_timer = current_time;                                  //Set timer_3 to current_time
    }
  } else if(last_ch3 == 1){                               //Input 10 is not high and changed from 1 to 0
    last_ch3 = 0;                                        //Remember current input state
    receiver_ch_3 = current_time - ch3_timer;         //Channel 3 is current_time - timer_3
  }
  
}
