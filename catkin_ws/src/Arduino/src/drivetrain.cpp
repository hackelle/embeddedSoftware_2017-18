#include "Arduino.h"
#include "drivetrain.h"

Drivetrain::Drivetrain(int le, int lf, int lb, int re, int rf, int rb) {
  pinMode(lf, OUTPUT);
  pinMode(le, OUTPUT);
  pinMode(lb, OUTPUT);

  pinMode(rf, OUTPUT);
  pinMode(re, OUTPUT);
  pinMode(rb, OUTPUT);

  this->Start();
}

void Drivetrain::Start() {
  digitalWrite(le, HIGH);
  digitalWrite(re, HIGH);
}

void Drivetrain::Move(int left, int right) {
  //restrict left and right to -255 to 255
  left = ((left < -255) ? -255 : left);
  left = ((left > 255) ? 255 : left);
  right = ((right < -255) ? -255 : right);
  right = ((right > 255) ? 255 : right);
  //write pins
  if (left < 0) {
    analogWrite(lb, -left);
    analogWrite(lf, 0);
  } else {
    analogWrite(lb, 0);
    analogWrite(lf, left);
  }
  if (right < 0) {
    analogWrite(rb, -right);
    analogWrite(rf, 0);
  } else {
    analogWrite(rb, 0);
    analogWrite(rf, right);
  }
}

void Drivetrain::Stop() {
  digitalWrite(le, LOW);
  digitalWrite(re, LOW);
}
