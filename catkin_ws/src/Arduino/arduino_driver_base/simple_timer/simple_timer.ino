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
double current_speed = 0;
double speed_adjust = 0;
double distance_adjust = 0;
int check_follow_time = 0;

 //global variable for left and right drivetrain speed
 int left = 0;
 int right = 0;

 //State of the machine
 int state = 3;  // 1->Start; 2->Move; 3->Stop; 4->Follow
 bool timeout = false;

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
 * Meassures the distance to the next obstacle based on ultrasonic sensor.
 * @return distance in cm
 */
int get_distance(){

  int duration = pulseIn(HC_SR04_ECHO, HIGH);
  int cm = (duration/2) / 29.1;

  return cm;
}

class NewHardware : public ArduinoHardware {
  public: NewHardware (): ArduinoHardware(&Serial1, 57600) {};
};
ros::NodeHandle_<NewHardware> nh;


void calculate_velocities(double x, double yaw){
  current_speed = x;
  if (x > TRACK_SPEED) {
    x = TRACK_SPEED;
  }
  if (yaw != 0) {
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
  TCNT5=0;
  double yaw = twist_msg.angular.z; // rad/s
  double x = twist_msg.linear.x;    //   m/s
  if (state == 2) {
    calculate_velocities(x, yaw);
  } else if (state == 4) {
    calculate_velocities(speed_adjust, yaw);
  }
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

  // Setting up timer 5 for timeout behaviour (1Hz)
  noInterrupts();
  TCCR5A = 0;
  TCCR5B = 0;
  TCNT5 = 0;
  OCR5A = 62499;
  TCCR5B |= (1 << WGM12); //CTC mode
  TCCR5B |= (1 << CS52);
  TIMSK5 |= (1 << OCIE5A);

  // Setting up timer 1 for firing pulses at 4Hz interval
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1 = 0;
  OCR1A = 15624;
  TCCR1B |= (1 << WGM12); //CTC mode
  TCCR1B |= (1 << CS12);
  TIMSK1 |= (1 << OCIE1A);
  interrupts();
}

ISR(TIMER5_COMPA_vect) {
  timeout = true;
}

ISR(TIMER1_COMPA_vect) {
  digitalWrite(HC_SR04_TRIGGER, LOW);
  delayMicroseconds(5);
  digitalWrite(HC_SR04_TRIGGER, HIGH);
  delayMicroseconds(20);
  digitalWrite(HC_SR04_TRIGGER, LOW);
  
  int duration = pulseIn(HC_SR04_ECHO, HIGH);
  int cm = (duration/2) / 29.1;
  distance_adjust = cm;
}

void Start() {
  digitalWrite(LCHB_100_1EN, HIGH);
  digitalWrite(LCHB_100_2EN, HIGH);
}

void Move() {
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

void Follow() {
  check_follow_time = millis();
  if (distance_adjust < get_distance()) {
    speed_adjust += current_speed/10;
  } else if (distance_adjust > get_distance()) {
    speed_adjust -= current_speed/10;
  }
  distance_adjust = get_distance();
  if (speed_adjust > current_speed) {
    speed_adjust = current_speed;
  } else if (speed_adjust < 0) {
    speed_adjust = 0;
  }
}

void Stop() {
  digitalWrite(LCHB_100_1EN, LOW);
  digitalWrite(LCHB_100_2EN, LOW);
}

void loop()
{
  nh.spinOnce();
  state_func();
  if (state == 3) {
    Stop();
  } else if (state == 1) {
    Start();
  } else if (state == 2 || state == 4) {
    Move();
  }
  delay(100);
}

void state_func() {
  if (distance_adjust < 20 || timeout == true) {
    state = 3;
  } else if (distance_adjust < 40) {
    Follow();
    state = 4;
  } else if (distance_adjust > 40 && (state == 3 || state == 4)) {
    state = 1;
  } else {
    state = 2;
  }
}
