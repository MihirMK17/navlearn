#define L298N_enA 9
#define L298N_in1 12
#define L298N_in2 13
#define right_encoder_phaseA 3
#define right_encoder_phaseB 5

unsigned int right_encoder_counter = 0;
String right_encoder_sign = "p";
double right_wheel_meas_vel = 0.0;

void setup() {
  // put your setup code here, to run once:
  pinMode(L298N_enA, OUTPUT);
  pinMode(L298N_in1, OUTPUT);
  pinMode(L298N_in2, OUTPUT);
  pinMode(right_encoder_phaseB, INPUT);

  attachInterrupt(digitalPinToInterrupt(right_encoder_phaseA), rightEncoderCallback, RISING);

  digitalWrite(L298N_in1, HIGH);
  digitalWrite(L298N_in2, LOW);

  Serial.begin(115200);
}

void loop() {
  right_wheel_meas_vel = 10 * right_encoder_counter * (60/385.0) * 0.10472;
  String encoder_read = right_encoder_sign + String(right_wheel_meas_vel);
  Serial.println(encoder_read);
  analogWrite(L298N_enA, 100);

  right_encoder_counter = 0;
  delay(100);
}

void rightEncoderCallback()
{
  right_encoder_counter++;
  if(digitalRead(right_encoder_phaseB) == HIGH)
  {
    right_encoder_sign = "p";
  }
  else
  {
    right_encoder_sign = "n";
  }
}

















