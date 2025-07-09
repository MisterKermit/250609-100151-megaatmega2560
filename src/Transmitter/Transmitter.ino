#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

int ce_pin = 7;
int csn_pin = 8;
RF24 radio(ce_pin, csn_pin);
bool success;
int buttonState;

int buttonPin = 3;
int ledPIN = 4;

const byte address[6] = "00001";

void setup() {
  Serial.begin(9600);
  pinMode(buttonPin, INPUT);
  pinMode(ledPIN, OUTPUT);
  radio.begin();
  radio.openWritingPipe(address);
  radio.setPALevel(RF24_PA_MIN);
  radio.stopListening();
}

void loop() {
  buttonState = digitalRead(buttonPin);
  // Serial.print("button state: ");
  // Serial.println(buttonState);
  success = radio.write(&buttonState, sizeof(buttonState));
  if (success) {
    digitalWrite(ledPIN, HIGH);
  } else {
    digitalWrite(ledPIN, LOW);
  }
  Serial.println(success);
  // const char text[] = "Hello World";
  delay(100);
}