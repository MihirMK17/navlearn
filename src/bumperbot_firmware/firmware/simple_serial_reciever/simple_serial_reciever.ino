#define LED_PIN 13

void setup() {
  // put your setup code here, to run once:
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  Serial.begin(115200);
}

void loop() {
  // put your main code here, to run repeatedly:
  if(Serial.available())
  {
    String str = Serial.readString();
    Serial.println("Received: [" + str + "]");
    int x = str.toInt();
    if (x == 0)
    {
      digitalWrite(LED_PIN, LOW);
    } else {
      digitalWrite(LED_PIN, HIGH);
    }
  }
}
