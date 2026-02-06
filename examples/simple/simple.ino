#include <Arduino.h>

// if passed in via -D MSF_TIME_LIB_DEBUG, if you are not using -D argument and just using Arduino
// IDE, just remove this and write #define MSF_TIME_LIB_DEBUG 1 directly
#ifndef MSF_TIME_LIB_DEBUG
#define MSF_TIME_LIB_DEBUG 1
#endif

#include <MSF-Time-Lib.h>

#define INPUT_PIN 3

bool readInput() { return digitalRead(INPUT_PIN) == LOW; }

MSFReceiver<1> msf(readInput);

void printDigits(int digits) {
  Serial.print(":");
  if (digits < 10) Serial.print('0');
  Serial.print(digits);
}

void setup() {
  Serial.begin(115200);
  pinMode(INPUT_PIN, INPUT_PULLUP);

  randomSeed(analogRead(0));
  while (analogRead(0) < 100) {
    delay(10);
    Serial.print(".");
  }

  Serial.println(F(">>> SYSTEM STARTUP"));
  Serial.println(F(">>> WAITING FOR RADIO SYNC (BLOCKING)"));
}

void loop() {
  while (1) {
    // get MSF time, this will block until we get a valid reading with correct checksum
    MSFData validData = msf.get_time_with_retry();
    Serial.print(F("RESULT: "));
    Serial.print(validData.year);
    Serial.print('-');
    if (validData.month < 10) Serial.print('0');
    Serial.print(validData.month);
    Serial.print('-');
    if (validData.day < 10) Serial.print('0');
    Serial.print(validData.day);
    Serial.print('T');
    if (validData.hour < 10) Serial.print('0');
    Serial.print(validData.hour);
    printDigits(validData.minute);
    printDigits(validData.second);
    Serial.println(F(" "));
  }
}
