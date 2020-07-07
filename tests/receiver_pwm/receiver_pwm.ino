/*
 * Tested in conjuction with Taranis delta mixing where endpoints are 50% of total
 */

byte CH1_PWM_PIN = 8; // CH1 throttle
byte CH2_PWM_PIN = 9; // CH2 left elevon
byte CH3_PWM_PIN = 10; // CH3 right elevon

int ch1_pwm_value;
int ch2_pwm_value;
int ch3_pwm_value;
 
void setup() {
  pinMode(CH1_PWM_PIN, INPUT);
  pinMode(CH2_PWM_PIN, INPUT);
  pinMode(CH3_PWM_PIN, INPUT);
  Serial.begin(115200);
}
 
void loop() {

  // Get the PWM values from the receiver
  ch1_pwm_value = pulseIn(CH1_PWM_PIN, HIGH);
  ch2_pwm_value = pulseIn(CH2_PWM_PIN, HIGH);
  ch3_pwm_value = pulseIn(CH3_PWM_PIN, HIGH);

  // Print the values
  Serial.print(ch1_pwm_value);
  Serial.print("\t");
  Serial.print(ch2_pwm_value);
  Serial.print("\t");
  Serial.println(ch3_pwm_value);
}
