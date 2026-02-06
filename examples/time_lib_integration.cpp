#include <Arduino.h>
#include <TimeLib.h>

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
    // put the time we got from MSF signal into TimeLib format and set the system time, now we can
    // use TimeLib functions to get the current time and print it in human readable format
    tmElements_t tElements_time = {validData.second, validData.minute, validData.hour,
                                   validData.day,    validData.month,  validData.year};
    setTime(makeTime(tElements_time));
    Serial.print(F("RESULT: "));
    Serial.print(year());
    Serial.print('-');
    if (month() < 10) Serial.print('0');
    Serial.print(month());
    Serial.print('-');
    if (day() < 10) Serial.print('0');
    Serial.print(day());
    Serial.print('T');
    if (hour() < 10) Serial.print('0');
    Serial.print(hour());
    printDigits(minute());
    printDigits(second());
    Serial.println(F(" "));
  }
}
