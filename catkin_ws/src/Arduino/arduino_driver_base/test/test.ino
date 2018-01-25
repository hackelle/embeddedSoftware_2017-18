
int LCHB_100_1REV = 7; // analog
int LCHB_100_1EN = 24; // digital
int LCHB_100_1FWD = 6; // analog
// right
int LCHB_100_2REV = 3; // analog
int LCHB_100_2EN = 25; // digital
int LCHB_100_2FWD = 2; // analog

void setup() {
  // Set motor pin to output.
  pinMode(LCHB_100_1REV, OUTPUT);
  pinMode(LCHB_100_1EN, OUTPUT);
  pinMode(LCHB_100_1FWD, OUTPUT);

  pinMode(LCHB_100_2REV, OUTPUT);
  pinMode(LCHB_100_2EN, OUTPUT);
  pinMode(LCHB_100_2FWD, OUTPUT);

  digitalWrite(LCHB_100_1EN, HIGH);
  digitalWrite(LCHB_100_2EN, HIGH);

  //Timer settings
  noInterrupts();
  TCCR3A = 0;
  TCCR3B = 0;
  TCNT3 = 0;
  OCR3A = 6249;
  TCCR3B |= (1 << WGM12); //CTC mode
  TCCR3B |= (1 << CS32);
  TIMSK3 |= (1 << OCIE3A);

  // Setting up timer 1 for firing pulses at 4Hz interval
  TCCR4A = 0;
  TCCR4B = 0;
  TCNT4 = 0;
  OCR4A = 124;
  TCCR4B |= (1 << WGM12); //CTC mode
  TCCR4B |= (1 << CS42);
  TIMSK4 |= (1 << OCIE4A);
  interrupts();
}

void loop() {
  analogWrite(LCHB_100_1FWD, 255);
  analogWrite(LCHB_100_2FWD, 255);
}
