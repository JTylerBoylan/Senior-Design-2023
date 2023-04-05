// motor ids
#define STEER 0
#define DRIVE 1

// input command bounds
#define DUTY_MAX 127
#define DUTY_MIN -128

// serial output bounds
#define SERIAL_MAX 127
#define SERIAL_MIN 1

// calibration params
#define CALIBRATE_SPEED_RATIO 0.5f
#define CALIBRATE_DELAY 150
#define ENCODER_MARGIN 50
#define DELTA_RANGE 1000

// pin definitions
#define INT1 6 // Y -> Y
#define INT2 7 // B -> W
#define HIGH_PIN 32

#define DEBUG_RATE 5000 // ms

// encoder global vars
byte state1 = 0b0;
int encoder_val = 0;

// motor values
int motor_commands[2] = {0, 0};

// calibration vars
int min_encoder_val = 0, max_encoder_val = 0;

void setup() {
  // begin serial coms
  Serial.begin(9600);
  Serial1.begin(9600);

  // run initializations
  pinMode(HIGH_PIN, OUTPUT);
  digitalWrite(HIGH_PIN, HIGH);
  initialize_encoder();
  calibrate_steering();
}

void loop() {
  // read from jetson
  read_commands();
  // run motors
  run_steering();
  run_drive();

  //debug
  print_debug();
}

void print_debug() {
  static long time_last = millis();
  if (millis() - time_last < DEBUG_RATE)
    return;
  Serial.println("----- DEBUG -----");
  Serial.print("Min Encoder val: ");
  Serial.println(min_encoder_val);
  Serial.print("Max Encoder val: ");
  Serial.println(max_encoder_val);
  Serial.print("Encoder val: ");
  Serial.println(encoder_val);
  Serial.print("Steer motor: ");
  Serial.println(motor_commands[STEER]);
  Serial.print("Drive motor: ");
  Serial.println(motor_commands[DRIVE]);
  time_last = millis();
}

void read_commands() {
  while (Serial.available() > 0) { // 8 bytes = 2 ints
    motor_commands[STEER] = Serial.parseInt();
    motor_commands[DRIVE] = Serial.parseInt();
    Serial.print("Updated commands to: ");
    Serial.print(motor_commands[STEER]);
    Serial.print(" & ");
    Serial.println(motor_commands[DRIVE]);
    while (Serial.available() > 0) Serial.read(); // flush serial
  }
  motor_commands[STEER] = motor_commands[STEER] > DUTY_MAX || motor_commands[STEER] < DUTY_MIN ? 0 : motor_commands[STEER];
  motor_commands[DRIVE] = motor_commands[DRIVE] > DUTY_MAX || motor_commands[DRIVE] < DUTY_MIN ? 0 : motor_commands[DRIVE];
}

void set_drive_speed(const int duty) {
  const int serial_duty = map(duty, DUTY_MIN, DUTY_MAX, SERIAL_MIN, SERIAL_MAX);
  Serial1.write(127 + serial_duty);
}

void set_steering_speed(const int duty) {
  const int serial_duty = map(duty, DUTY_MIN, DUTY_MAX, SERIAL_MIN, SERIAL_MAX);
  Serial1.write(serial_duty);
}

void run_steering() {
  steer_to_position(motor_commands[STEER]);
}

void run_drive() {
  set_drive_speed(motor_commands[DRIVE]);
}

bool steer_to_position(int duty_pos) {
  const int des_pos = map(duty_pos, DUTY_MIN, DUTY_MAX, min_encoder_val, max_encoder_val);
  const int delta = des_pos - encoder_val;
  const int duty = delta_to_duty(delta);
  if (abs(delta) > ENCODER_MARGIN) {
    set_steering_speed(duty);
    return false;
  } else {
    set_steering_speed(0);
    return true;
  }
}

int delta_to_duty(int delta) {
  int duty = map(delta, -DELTA_RANGE, DELTA_RANGE, DUTY_MAX, DUTY_MIN);
  if (duty > DUTY_MAX)
    duty = DUTY_MAX;
  else if (duty < DUTY_MIN)
    duty = DUTY_MIN;
  return duty;
}

void calibrate_steering() {

  delay(5000);

  Serial.println("Starting calibration...");

  int encoder_last = encoder_val;

  // find min encoder val
  set_steering_speed(CALIBRATE_SPEED_RATIO * DUTY_MAX);
  delay(CALIBRATE_DELAY);
  while(abs(encoder_val - encoder_last) > ENCODER_MARGIN) {
    encoder_last = encoder_val;
    delay(CALIBRATE_DELAY);
  }
  min_encoder_val = encoder_val + ENCODER_MARGIN;

  Serial.print("Min encoder value set to: ");
  Serial.println(min_encoder_val);

  // find max encoder val
  set_steering_speed(CALIBRATE_SPEED_RATIO * DUTY_MIN);
  delay(CALIBRATE_DELAY);
  while(abs(encoder_val - encoder_last) > ENCODER_MARGIN) {
    encoder_last = encoder_val;
    delay(CALIBRATE_DELAY);
  }
  max_encoder_val = encoder_val - ENCODER_MARGIN;

  Serial.print("Max encoder value set to: ");
  Serial.println(max_encoder_val);

  Serial.println("Going to zero...");

  // go to zero position
  while(!steer_to_position(0));

  Serial.println("Calibration done.");

  delay(5000);
}

/******************************************************* 
 * ****************** ENCODER FUNCTIONS ************** *
 *******************************************************/

void encoder1CHA(void) {

  if ((state1 == 0) && (digitalRead(INT1) == 1)) {
    state1 = 2;
    encoder_val = encoder_val - 1;
  }
  else if ((state1 == 2) && (digitalRead(INT1) == 0)) {
    state1 = 0;
    encoder_val = encoder_val + 1;
  }
  else if ((state1 == 1) && (digitalRead(INT1) == 1)) {
    state1 = 3;
    encoder_val = encoder_val + 1;
  }
  else if ((state1 == 3) && (digitalRead(INT1) == 0)) {
    state1 = 1;
    encoder_val = encoder_val - 1;
  }

}

void encoder1CHB(void) {

  if ((state1 == 0) && (digitalRead(INT2) == 1)) {
    state1 = 1;
    encoder_val = encoder_val + 1;
  }
  else if ((state1 == 3) && (digitalRead(INT2) == 0)) {
    state1 = 2;
    encoder_val = encoder_val + 1;
  }
  else if ((state1 == 2) && (digitalRead(INT2) == 1)) {
    state1 = 3;
    encoder_val = encoder_val - 1;
  }
  else if ((state1 == 1) && (digitalRead(INT2) == 0)) {
    state1 = 0;
    encoder_val = encoder_val - 1;
  }

}

void initialize_encoder(void) {
  pinMode(INT1, INPUT_PULLUP);
  pinMode(INT2, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(INT1), encoder1CHA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(INT2), encoder1CHB, CHANGE);
}

