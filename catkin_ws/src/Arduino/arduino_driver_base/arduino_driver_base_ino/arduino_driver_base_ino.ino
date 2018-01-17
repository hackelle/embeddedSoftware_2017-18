/*
 * rosserial Sub
 * Toggles LED on topic toggle_led (for tests).
 * Reads an executes twist_msgs as defined in ros standard.
 */

#include <ros.h>
#include <std_msgs/Empty.h>
#include "geometry_msgs/Twist.h"

 // max. speed of tracks = 18 cm/s
double RIGHT_TRACK_MAX_FWRD = 0.18;
double RIGHT_TRACK_MAX_BACK = 0.18;
double LEFT_TRACK_MAX_FWRD = 0.18;
double LEFT_TRACK_MA
X_BACK = 0.18;
 // max. rot = 4.5 rad/s => 1.4 sec/rotation

// motor
// left
int LCHB_100_1REV = 7; // analog
int LCHB_100_1EN = 24; // digital
int LCHB_100_1FWD = 6; // analog
// right
int LCHB_100_2REV = 3; // analog
int LCHB_100_2EN = 25; // digital
int LCHB_100_2FWD = 2; // analog

// bluetooth
int HC_05_TX = 18; // digital
int HC_05_RX = 19; // digital

// ultrasonic sensor
int HC_SR04_TRIGGER = 23; // digital
int HC_SR04_ECHO = 22; // digital

// led
int YELLOW_LED = 13; // analog

/**
 * Writes the speed to the motors
 * @param left speed of the left motor
 * @param right speed of the right motor
 */
void set_motor(int left, int right){
  // restrict left and right to -255 to 255
  left = ((left < -255) ? -255 : left);
  left = ((left > 255) ? 255 : left);
  right = ((right < -255) ? -255 : right);
  right = ((right > 255) ? 255 : right);
  // write pins
  if (left < 0) {
    analogWrite(LCHB_100_1REV, -left);
    analogWrite(LCHB_100_1FWD, 0);
  } else {
    analogWrite(LCHB_100_1REV, 0);
    analogWrite(LCHB_100_1FWD, left);
  }
  if (right < 0) {
    analogWrite(LCHB_100_2REV, -right);
    analogWrite(LCHB_100_2FWD, 0);
  } else {
    analogWrite(LCHB_100_2REV, 0);
    analogWrite(LCHB_100_2FWD, right);
  }
}

/**
 * Meassures the distance to the next obstacle based on ultrasonic sensor.
 * @return distance in cm
 */
int get_distance(){
  // trigger
  // make sure it's low
  digitalWrite(HC_SR04_TRIGGER, LOW);
  delay(5);
  digitalWrite(HC_SR04_TRIGGER, HIGH);
  delay(20);
  digitalWrite(HC_SR04_TRIGGER, LOW);

  int duration = pulseIn(HC_SR04_ECHO, HIGH);
  int cm = (duration/2) / 29.1;

  //cm = ((cm < 0) ? 0 : cm);

  return cm;
}

class NewHardware : public ArduinoHardware {
  public: NewHardware (): ArduinoHardware(&Serial1, 57600) {};
};
ros::NodeHandle_<NewHardware> nh;

void blink(){
  digitalWrite(YELLOW_LED, HIGH);
  delay(30);
  digitalWrite(YELLOW_LED, LOW);
  delay(30);
  digitalWrite(YELLOW_LED, HIGH);
  delay(30);
  digitalWrite(YELLOW_LED, LOW);
  delay(30);
}

void calculate_velocities(double x, double yaw, int* left, int* right){
  *left = (int) x;
  *right = (int) x;
}

void led_messageCb( const std_msgs::Empty& toggle_msg){
  digitalWrite(YELLOW_LED, HIGH-digitalRead(13));   // blink the led
}

void twist_messageCb( const geometry_msgs::Twist& twist_msg){
  blink();
  double yaw = twist_msg.angular.z; // rad/s
  double x = twist_msg.linear.x;    //   m/s
  int* left, right;
  calculate_velocities(x, yaw, left, right);
  set_motor(*left, *right);  // blink the led
}

ros::Subscriber<std_msgs::Empty> led_sub("toggle_led", led_messageCb);
ros::Subscriber<geometry_msgs::Twist> twist_sub("cmd_vel", twist_messageCb);

void setup()
{
  pinMode(13, OUTPUT);
  nh.initNode();
  nh.subscribe(led_sub);
  nh.subscribe(twist_sub);

  // Set motor pin to output.
  pinMode(LCHB_100_1REV, OUTPUT);
  pinMode(LCHB_100_1EN, OUTPUT);
  pinMode(LCHB_100_1FWD, OUTPUT);

  pinMode(LCHB_100_2REV, OUTPUT);
  pinMode(LCHB_100_2EN, OUTPUT);
  pinMode(LCHB_100_2FWD, OUTPUT);

  // enable both motors
  digitalWrite(LCHB_100_1EN, HIGH);
  digitalWrite(LCHB_100_2EN, HIGH);

  pinMode(HC_SR04_TRIGGER, OUTPUT);
  pinMode(HC_SR04_ECHO, INPUT);

  Serial.begin(9600);

  pinMode(YELLOW_LED, OUTPUT);
}

void loop()
{
  nh.spinOnce();
  int distance = get_distance();
  if (distance < 40){
    analogWrite(YELLOW_LED, 255);
    digitalWrite(LCHB_100_1EN, LOW);
    digitalWrite(LCHB_100_2EN, LOW);
    Serial.println("Safety stop!");
  } else {
    analogWrite(YELLOW_LED, 0);
    digitalWrite(LCHB_100_1EN, HIGH);
    digitalWrite(LCHB_100_2EN, HIGH);
  }
  delay(100);
}
