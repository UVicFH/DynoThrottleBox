/**
 * UVIC FORMULA HYBRID 2017
 * 
 * This is temporary code to control the throttle body while the mechanical drive train
 * is running on its own. It reads the position of a potentiometer (which simulates the
 * gas pedal) as well as the throttle position sensor (TPS) and drives the motor (PWM_OUT)
 * to turn the throttle body accordingly.
 * 
 * Author(s):
 * Chad McColm
 * Taylor Murias
 */

 /* Input pins */
#define POT_IN A0 // potentiometer
#define TPS_IN A5 // throttle position sensor

/* Drives throttle body motor */
#define PWM_OUT 10

/* Config pins, should be kept constant */
#define INA_OUT 0
#define INB_OUT 2
#define ENA_OUT 4
#define ENB_OUT 6

/* PWM values for driving the motor */
#define FULLY_CLOSED 0
#define FULLY_OPEN 255
#define FIXED 50 // Not enough to move it either way
#define PUSH_OPEN 100
#define PUSH_CLOSED 1

void setup() {
  // Serial stream for logging values
  Serial.begin(9600);

  // Analog input pins
  pinMode(POT_IN, INPUT);
  pinMode(TPS_IN, INPUT);

  // Output pins
  pinMode(PWM_OUT, OUTPUT);
  pinMode(INA_OUT, OUTPUT);
  pinMode(INB_OUT, OUTPUT);
  pinMode(ENA_OUT, OUTPUT);
  pinMode(ENB_OUT, OUTPUT);

  // Set motor to spin in direction A
  digitalWrite(INA_OUT, LOW);
  digitalWrite(INB_OUT, LOW);

  // Enable all motor bridges
  digitalWrite(ENA_OUT, HIGH);
  digitalWrite(ENB_OUT, HIGH);
}

void loop() {

  // Get potentiometer value scaled to 0-100
  double throttle_request = read_pot();

  // Get TPS value scaled to 0-100
  double throttle_pos = read_tps();

  // Calculate how much the motor needs to be moved
  double pwm_val = calculate_pwm_val(throttle_request, throttle_pos);

  // Move the motor
  analogWrite(PWM_OUT, pwm_val);
}

/*
 * Read the analog pin that the potentiometer is connected to and
 * scale it to be from 0 - 100.
 */
double read_pot() {
  double val = analogRead(POT_IN)/1024.0 * 100;

  if (val < 0)
    return 0;
  else if (val > 100)
    return 100;

  return val;
}

/*
 * Read the analog pin that the throttle position sensor is connected to
 * and scale it to be from 0 - 100.
 */
double read_tps() {
  double val = (analogRead(TPS_IN) - 265.0) / 5.44;

  if (val < 0)
    return 0;
  else if (val > 100)
    return 100;

  return val;
}

/*
 * Given the position we want the throttle body to be in and the value
 * it's currently in, calculates the PWM value that will drive the motor
 * toward the desired position. Parameters are values from 0 (fully closed)
 * to 100 (wide open).
 */
double calculate_pwm_val(double throttle_request, double throttle_pos) {

  // How much the motor needs to be moved to achive throttle_request
  double diff = throttle_request - throttle_pos;

  // Determine how much motor speed is needed for the adjustment
  if (throttle_request < 5) {
    // Go straight to closed as fast as possible
    return FULLY_CLOSED;
  } else if (throttle_request > 95) {
    // Go straight to open as fast as possible
    return FULLY_OPEN;
  } else if (abs(diff) > 5) {
    // Adjustment is needed, push the motor partially
    return (diff > 0) ? PUSH_OPEN : PUSH_CLOSED;
  } else {
    // Keep it where it is
    return FIXED;
  }
}

