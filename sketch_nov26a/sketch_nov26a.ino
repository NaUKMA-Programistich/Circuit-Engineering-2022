int redPin = 13;

void setup() {
  // initialize Leds
  pinMode(redPin, OUTPUT); 

}

void loop() {
  digitalWrite(redPin, HIGH);
  delay(100);
  digitalWrite(redPin, LOW);
  delay(100);

}