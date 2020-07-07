byte CH2_PWM_PIN = 18; // CH2 left elevon
byte CH3_PWM_PIN = 19; // CH3 right elevon
 
int ch2_pwm_value;
int ch3_pwm_value;
 
void setup() {
  pinMode(CH2_PWM_PIN, INPUT);
  pinMode(CH3_PWM_PIN, INPUT);
  Serial.begin(115200);
}
 
void loop() {
  ch2_pwm_value = pulseIn(CH2_PWM_PIN, HIGH);
  ch3_pwm_value = pulseIn(CH3_PWM_PIN, HIGH);

  // Print the values
  Serial.print(ch2_pwm_value);
  Serial.print("/t");
  Serial.println(ch3_pwm_value);
}
