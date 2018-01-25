/*
 * rosserial Sub
 * Toggles LED on topic toggle_led (for tests).
 * Reads an executes twist_msgs as defined in ros standard.
 */

#include <ros.h>
#include <std_msgs/Empty.h>
#include "geometry_msgs/Twist.h"

 // max. speed of tracks = 18 cm/s
double TRACK_SPEED = 0.18;

 //global variable for left and right drivetrain speed
 int left = 0;
 int right = 0;

 //State of the machine
 int state = 0;  // 1->Start; 2->Move; 3->Stop

 //Variables for controlling the timeout behaviour of the subscribed topic
 double last_sub = 0;
 double sub_timeout = 1000;

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
void Move(){
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
  delayMicroseconds(5);
  digitalWrite(HC_SR04_TRIGGER, HIGH);
  delayMicroseconds(20);
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


void calculate_velocities(double x, double yaw){
  if (x > TRACK_SPEED) {
    x = TRACK_SPEED;
  }
  if (yaw != 0 && x!=0) {
    left = (x+0.2827/(4*PI/yaw))*255/TRACK_SPEED;
    right = (x-0.2827/(4*PI/yaw))*255/TRACK_SPEED;
  } else if(yaw != 0 && x==0) {
    left = (x+0.2827/(4*PI/yaw))*255/TRACK_SPEED+50;
    right = (x-0.2827/(4*PI/yaw))*255/TRACK_SPEED-50;
  }
   else {
    left = right = x*255/TRACK_SPEED;
  }
}

void led_messageCb( const std_msgs::Empty& toggle_msg){
  digitalWrite(YELLOW_LED, HIGH-digitalRead(13));   // blink the led
}

void twist_messageCb( const geometry_msgs::Twist& twist_msg){
  last_sub = millis();
  double yaw = twist_msg.angular.z; // rad/s
  double x = twist_msg.linear.x;    //   m/s
  calculate_velocities(x, yaw);
}

ros::Subscriber<std_msgs::Empty> led_sub("toggle_led", led_messageCb);
ros::Subscriber<geometry_msgs::Twist> twist_sub("cmd_vel", twist_messageCb);

void setup()
{
  pinMode(YELLOW_LED, OUTPUT);
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


  pinMode(HC_SR04_TRIGGER, OUTPUT);
  pinMode(HC_SR04_ECHO, INPUT);

  Serial.begin(9600);

  Start();
}

void Start() {
  digitalWrite(LCHB_100_1EN, HIGH);
  digitalWrite(LCHB_100_2EN, HIGH);
}

void Stop() {
  digitalWrite(LCHB_100_1EN, LOW);
  digitalWrite(LCHB_100_2EN, LOW);
}

void loop()
{
  nh.spinOnce();
  int distance = get_distance();
  if (distance < 40 || millis()-last_sub > sub_timeout) {
    analogWrite(YELLOW_LED, 255);
    state = 3;
    Stop();
  } else if (distance > 40 && state == 3) {
    analogWrite(YELLOW_LED, 0);
    state = 1;
    Start();
  } else {
    analogWrite(YELLOW_LED, 0);
    state = 2;
    Move();
  }
  delay(100);
}
