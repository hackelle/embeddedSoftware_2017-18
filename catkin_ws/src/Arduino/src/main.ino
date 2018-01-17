/*
 * rosserial Sub
 * Toggles LED on topic toggle_led (for tests).
 * Reads an executes twist_msgs as defined in ros standard.
 */

#include <ros.h>
#include <Fsm.h>
#include <std_msgs/Empty.h>

#include "geometry_msgs/Twist.h"
#include "sensor.h"
#include "drivetrain.h"

 // max. speed of tracks = 18 cm/s
double RIGHT_TRACK_MAX_FWRD = 0.18;
double RIGHT_TRACK_MAX_BACK = 0.18;
double LEFT_TRACK_MAX_FWRD = 0.18;
double LEFT_TRACK_MAX_BACK = 0.18;
 // max. rot = 4.5 rad/s => 1.4 sec/rotation

Sensor *sensor = NULL;
Drivetrain *drivetrain = NULL;

int left = 0;
int right = 0;

int state = 0;  // 1->Start; 2->Move; 3->Stop
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

void calculate_velocities(double x, double yaw){
  left = (x+0.2827/(4*PI/yaw))*255/0.18;
  right = (x-0.2827/(4*PI/yaw))*255/0.18;
}

void led_messageCb( const std_msgs::Empty& toggle_msg){
  digitalWrite(YELLOW_LED, HIGH-digitalRead(13));   // blink the led
}

void twist_messageCb( const geometry_msgs::Twist& twist_msg){
  blink();
  double yaw = twist_msg.angular.z; // rad/s
  double x = twist_msg.linear.x;    //   m/s
  calculate_velocities(x, yaw);
}

ros::Subscriber<std_msgs::Empty> led_sub("toggle_led", led_messageCb);
ros::Subscriber<geometry_msgs::Twist> twist_sub("cmd_vel", twist_messageCb);

void setup()
{
  pinMode(13, OUTPUT);
  nh.initNode();
  nh.subscribe(led_sub);
  nh.subscribe(twist_sub);

  sensor = new Sensor(HC_SR04_TRIGGER, HC_SR04_ECHO);
  drivetrain = new Drivetrain(LCHB_100_1EN, LCHB_100_1FWD, LCHB_100_1REV, LCHB_100_2EN, LCHB_100_2FWD, LCHB_100_2REV);
  state = 1;

  Serial.begin(9600);

  pinMode(YELLOW_LED, OUTPUT);
}

void loop()
{
  nh.spinOnce();
  int distance = sensor->get_distance();
  if (distance < 40){
    analogWrite(YELLOW_LED, 255);
    state = 3;
    drivetrain->Stop();
    Serial.println("Safety stop!");
  } else if (distance > 40 && state == 3) {
    analogWrite(YELLOW_LED, 0);
    state = 1;
    drivetrain->Start();
  } else {
    state = 2;
    drivetrain->Move(left, right);
  }
  delay(100);
}
