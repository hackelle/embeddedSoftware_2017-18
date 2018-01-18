#include "Arduino.h"
#include "sensor.h"

const unsigned int interval = 29.1;

Sensor::Sensor(const int trig, const int echo) {
  pinMode(trig, OUTPUT);
  pinMode(echo, INPUT);
}

int Sensor::get_distance() {
  digitalWrite(this->trig, LOW);
  delay(5);
  digitalWrite(this->trig, HIGH);
  delay(20);
  digitalWrite(this->trig, LOW);

  int duration = pulseIn(this->echo, HIGH);
  int cm = (duration/2)/interval;

  return cm;
}
