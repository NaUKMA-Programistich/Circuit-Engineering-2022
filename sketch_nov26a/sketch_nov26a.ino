// Init button
#define PIN_BTN A5

// Init RFID
#include <SPI.h>    
#include <MFRC522.h>
int uidAccess[] = { 17, 229, 32, 29 };

#define PIN_RFID_RST 6
#define PIN_RFID_SDA 7
MFRC522 rfid(PIN_RFID_SDA, PIN_RFID_RST);

// Init diod
#define PIN_DIOD_RED A0
#define PIN_DIOD A1
#define PIN_DIOD_GREEN A2

// Init servo
#include <Servo.h>
Servo servo;
#define PIN_SERVO A4
bool isOpenDoor = false;

// Init GPS
#include <SoftwareSerial.h>
#include <TinyGPS.h>
#define PIN_GPS_TX 2
#define PIN_GPS_RX 3
SoftwareSerial gpsSerial(PIN_GPS_TX, PIN_GPS_RX);
float gpslat, gpslon = 0.0;
TinyGPS gps;
char* gpsData = "";

// Init GSM
#define PIN_GSM_TX 4
#define PIN_GSM_RX 5
SoftwareSerial gsmSerial(PIN_GSM_TX, PIN_GSM_RX);

// Init watchdog
#include <avr/wdt.h>

void setup() {
  Serial.begin(9600);
  Serial.println("Start setup");

  setupGPS();
  setupServo();
  setupDiod();
  setupButton();
  setupRFID();
}

void setupServo() {
  servo.attach(PIN_SERVO);
  servo.write(0); // start position - door closed
}

void setupDiod() {
  pinMode(PIN_DIOD_RED, OUTPUT);
  pinMode(PIN_DIOD_GREEN, OUTPUT);
  pinMode(PIN_DIOD, OUTPUT);

  redLight(); // on start - door closed
}

void setupButton() {
  pinMode(PIN_BTN, INPUT_PULLUP);
}

void setupWatchDog() {
  wdt_enable(WDTO_4S); // 4 seconds waiting
}

void setupGPS() {
  gpsSerial.begin(9600);
}

void setupRFID() {
  SPI.begin();
  rfid.PCD_Init();
}

void loop() {
  processGPS();
  processButton();
  processRFID();
  processReset();
}

void processGPS() {
  gpsSerial.listen();
  while (gpsSerial.available() > 0) {
    if (isOpenDoor) {
      Serial.write(gpsSerial.read());
    }
  }
}

void processReset() {
  wdt_reset();
}

void processButton() {
  int buttonState = digitalRead(PIN_BTN);
  if (!buttonState) {
    Serial.println("Button Click");
    if (isOpenDoor) {
      closeDoor();
    }
    else {
      openDoor();
    }
    delay(200);
  }
}

void processRFID() {
  resetRFID();
  readRFID();
}

void resetRFID() {
  static uint32_t rfidRebootTimer = millis();
  if (millis() - rfidRebootTimer > 500) {
    rfidRebootTimer = millis();
    digitalWrite(PIN_RFID_RST, HIGH);
    delay(1);
    digitalWrite(PIN_RFID_RST, LOW);
    rfid.PCD_Init();
  }
}

void readRFID() {
  if (rfid.PICC_IsNewCardPresent() and rfid.PICC_ReadCardSerial()) {

    Serial.print("UID: ");
    for (uint8_t i = 0; i < 4; i++) {
      Serial.print(rfid.uid.uidByte[i]);
      Serial.print(", ");
    }
    Serial.println();
    processReadUID();
    delay(1500);
  }
}

void processReadUID() {
    int uidSize = rfid.uid.size;
    uint8_t* tag = rfid.uid.uidByte;
    bool isCorrectCard = isCorrectUID(tag, uidSize);

    while (true) {

      if (isCorrectCard && !isOpenDoor) {
        openDoor();
        break;
      }
      if (!isCorrectCard && !isOpenDoor) {
        blimRed();
        break;
      }
      break;
    }
}

bool isCorrectUID(uint8_t *tag, uint8_t size) {
  if (size != sizeof(uidAccess) / sizeof(int)) return false;
  for (int i = 0; i < size; i++) {
    if (tag[i] != uidAccess[i]) return false;
  }
  return true;
}

void closeDoor() {
  redLight();
  isOpenDoor = false;
  servo.write(0); // start position - door closed
}

void openDoor() {
  greenLight();
  isOpenDoor = true;
  servo.write(180); // end position - door opened
}

void greenLight() {
  analogWrite(PIN_DIOD, 255);
  analogWrite(PIN_DIOD_GREEN, 0);
  analogWrite(PIN_DIOD_RED, 255);
}

void redLight() {
  analogWrite(PIN_DIOD, 255);
  analogWrite(PIN_DIOD_RED, 0);
  analogWrite(PIN_DIOD_GREEN, 255);
}

void offLight() {
  analogWrite(PIN_DIOD, 255);
  analogWrite(PIN_DIOD_RED, 255);
  analogWrite(PIN_DIOD_GREEN, 255);
}

void blimRed() {
  for(int i = 0; i < 7; i++) {
    offLight();
    delay(50);
    redLight();
    delay(50);
  }
  redLight();
}

