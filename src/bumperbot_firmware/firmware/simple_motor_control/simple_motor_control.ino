#define L298N_enA 9
#define L298N_in1 12
#define L298N_in2 13

double cmd = 0.0;

void setup() {
  // put your setup code here, to run once:
  pinMode(L298N_enA, OUTPUT);
  pinMode(L298N_in1, OUTPUT);
  pinMode(L298N_in2, OUTPUT);

  digitalWrite(L298N_in1, HIGH);
  digitalWrite(L298N_in2, LOW);

  Serial.begin(115200);
}

void loop() {
  // put your main code here, to run repeatedly:
  if(Serial.available())
  {
    cmd = Serial.readString().toDouble();
  }

  analogWrite(L298N_enA, cmd*100);
}
