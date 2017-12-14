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
  } else {
    analogWrite(LCHB_100_1FWD, left);
  }
  if (right < 0) {
    analogWrite(LCHB_100_2REV, -right);
  } else {
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

void setup(){
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

void loop(){
  int distance = get_distance();
  if (distance > 40){
    set_motor(255, 255);
    analogWrite(YELLOW_LED, 0);
  } else {
    analogWrite(YELLOW_LED, 255);
    set_motor(0, 0);
  }
  Serial.println(distance);
  delay(100);
}
