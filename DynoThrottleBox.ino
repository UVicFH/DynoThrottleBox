/**
 * UVIC FORMULA HYBRID 2017-2018
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

/* PID Setup Parameters
 * 4, 30, 0.01 works, but finer calibration could be done. */
double Kp = 4;
double Ki = 30;
double Kd = 0.01;

/* PID global variables (across iterations) */
double integral = 0;
int oldtime = 0;
double error = 0;

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
  digitalWrite(INA_OUT, HIGH);
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

  // Limit the frequency of this loop.
  // Letting it run too fast results in too small dt values for integral
  // and derivative calculations, which results in high error.
  delay(10);
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
  double val = (analogRead(TPS_IN) - 280.0) / 5.85;

  if (val < 0)
    return 0;
  else if (val > 100)
    return 100;

  return val;
}

/*
 * Given the position we want the throttle body to be in and the value
 * it's currently in, uses PID to calculate the PWM value that will drive
 * the motor toward the desired position. Parameters are values from 0
 * (fully closed) to 100 (wide open).
 */
double calculate_pwm_val(double throttle_request, double tps_feedback) {

  // Save the old error for derivative calculation
  double olderror = error;
  
  // Determine the error between desired and current
  error = throttle_request - tps_feedback;

  // Record the current time to calculate the time derivative
  int runtime = millis();

  // Find the current rate of change of error w.r.t. time
  double dt = (runtime - oldtime) / 1000.0;
  double derivative = (error - olderror) / dt;

  // Set the oldtime to be the new runtime
  oldtime = runtime;

  // Add the error to the integral of the errors
  integral += error*dt;

  // Calculate the pwm request based on PID coeffecients
  double pwm_request = error*Kp + integral*Ki + derivative*Kd;

  // Minimum of request is 0
  if (pwm_request < 0) pwm_request = 0;

  // Override request if throttle is requested closed or wide open
  if (throttle_request < 0.5) pwm_request = 0;
  else if (throttle_request > 99.5) pwm_request = 255;

  return pwm_request;
}

