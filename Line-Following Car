// Base code.
//
// *  NOTE: this code will do only three things:
// *    --rotate one wheel, and
// *    --blink the right front mainboard LED.
// *
// *  You will need to add more code to
// *  make the car do anything useful.
//

#include <ECE3.h>
// #include <ECE3_LCD7.h>

uint16_t sensorValues[8];  // right -> left, 0 -> 7
int updatedSensorValues[8];
int minimums[] = { 619, 641, 642, 619, 619, 711, 688, 683 };          // Used for normalization, stolen from callibration data
int maximums[] = { 1467, 1859, 1858, 1277, 1372, 1730, 1812, 1817 };  // Used for normalization, stolen from callibration data

const int left_nslp_pin = 31;  // nslp ==> awake & ready for PWM
const int left_dir_pin = 29;
const int right_pwm_pin = 39;
const int left_pwm_pin = 40;

const int right_nslp_pin = 11;
const int right_dir_pin = 30;

const int LED_RF = 41;

// Global variables for PD controller
int previousError = 0;
int currentError = 0;
  int baseSpeed = 40;  // Base motor speed (adjust as needed), from 0 to 255
bool donut = false;
int blackLineCounter = 0;
bool completedDonut = false; // checks if car should be on route back
bool switchWeighting = false;
bool previousBlack = false;   // For phantom shit. Need to fulfill condition that you hit all black twice before you do the donut.


// ************************** ERROR CALCULATION **************************
// Currently without normalization.

int findError() {

  // All black counter
  int counter = 0;
  for (int i = 0; i < 8; i++) {
    if (sensorValues[i] > 1900) {
      counter++;
    }
  }
  
  if (counter >= 7 && previousBlack == true && completedDonut == false) {
    donut = true;
    previousBlack = false;
  }
  else if (counter >= 7 && previousBlack == false) {
    previousBlack = true;
  }
  else {
    previousBlack = false;
  }

  // Normalize the data (divide every value by 1000 divided by the maximum).
  for (int i = 0; i < 8; i++) {
    updatedSensorValues[i] = (sensorValues[i] - minimums[i]) * 1000.00 / (maximums[i]);
  }

  // Do Fusion Curves

  if (switchWeighting == false) {
    // Errors value with left sensors weighted more
    int errorValue = (-15 * updatedSensorValues[0]) + (-14 * updatedSensorValues[1]) + (-12 * updatedSensorValues[2]) + (-8 * updatedSensorValues[3]) + (26 * updatedSensorValues[4]) + (37 * updatedSensorValues[5]) + (45 * updatedSensorValues[6]) + (46 * updatedSensorValues[7]);
    errorValue /= 8;
    return errorValue;

  }
  else {
    // Errors value with right sensors weighted more
    int errorValue = (-26 * updatedSensorValues[0]) + (-25 * updatedSensorValues[1]) + (-24 * updatedSensorValues[2]) + (-20 * updatedSensorValues[3]) + (8 * updatedSensorValues[4]) + (12 * updatedSensorValues[5]) + (14 * updatedSensorValues[6]) + (15 * updatedSensorValues[7]);
    errorValue /= 8;
    return errorValue;
  }
}

// ************************** PD CONTROLLER **************************

void PDController() {
  currentError = findError();  // Updates global var currentError with the error from the sensor reading

  // Kp will be the (error margin) * (base speed) = Kp * (max error value)
  double Kp = 0.5 * baseSpeed * 6 / 1700;  // May need to change numbers lateer! 
  double Kd = Kp / 5;                  // Can adjust later!
  int changeInError = currentError - previousError;

  // Insert steering adjustments with wheels based on Kp , Kd, and changeInError calculations

  // Calculate P and D term
  float P_term = Kp * currentError;
  float D_term = Kd * (changeInError);
  float steering = P_term + D_term;

  // Apply steering function
  applySteering(steering);

  previousError = currentError;  // Set the previous error to be the value in current error
}

void applySteering(float steeringValue) {

  int leftMotorSpeed = baseSpeed - steeringValue;
  int rightMotorSpeed = baseSpeed + steeringValue;

  // Logic for the donut (will spin the donut around 180 degrees and move it forward)
  if (donut == true && completedDonut == false) {
    blackLineCounter++;
    if (blackLineCounter == 2) {
      analogWrite(left_pwm_pin, 0);
      analogWrite(right_pwm_pin, 0);
      delay(10000);
    }
    int turnSpeed = 40;
    digitalWrite(LED_RF,HIGH);  // LED for testing
    // Turns it around (one wheel reverses one doesn't);
    digitalWrite(left_dir_pin, HIGH);
    digitalWrite(right_dir_pin, LOW);
    analogWrite(left_pwm_pin, turnSpeed);
    analogWrite(right_pwm_pin, turnSpeed);
    digitalWrite(LED_RF,HIGH);
    delay(1800);
    digitalWrite(left_dir_pin,LOW); // change left wheel to go forward
    analogWrite(left_pwm_pin, turnSpeed);
    analogWrite(right_pwm_pin, turnSpeed);
    delay(500);
    donut = false;
    completedDonut = false;
    switchWeighting = true;
  }

  if (leftMotorSpeed < 0) {
    rightMotorSpeed = rightMotorSpeed / 5;
    leftMotorSpeed = abs(leftMotorSpeed);
    leftMotorSpeed = baseSpeed / 1.55;
    digitalWrite(left_dir_pin, HIGH);
  } else {
    digitalWrite(left_dir_pin, LOW);
  }

  if (rightMotorSpeed < 0) {
    leftMotorSpeed = leftMotorSpeed / 5;
    rightMotorSpeed = abs(rightMotorSpeed);
    rightMotorSpeed = baseSpeed / 1.55;
    digitalWrite(right_dir_pin, HIGH);
  } else {
    digitalWrite(right_dir_pin, LOW);
  }

  // Apply motor speeds to corresponding pins (adjust if needed)
  analogWrite(left_pwm_pin, leftMotorSpeed);
  analogWrite(right_pwm_pin, rightMotorSpeed);

}

///////////////////////////////////
void setup() {

  delay(2000);
  // put your setup code here, to run once:
  pinMode(left_nslp_pin, OUTPUT);
  pinMode(left_dir_pin, OUTPUT);
  pinMode(left_pwm_pin, OUTPUT);

  digitalWrite(left_dir_pin, LOW);
  digitalWrite(left_nslp_pin, HIGH);

  pinMode(right_nslp_pin, OUTPUT);
  pinMode(right_dir_pin, OUTPUT);
  pinMode(right_pwm_pin, OUTPUT);

  digitalWrite(right_dir_pin, LOW);
  digitalWrite(right_nslp_pin, HIGH);

  pinMode(LED_RF, OUTPUT);

  ECE3_Init();

/* commented this out
  // set the data rate in bits/second for serial data transmission
  Serial.begin(9600);
  delay(2000);  // Wait 2 seconds before starting

*/

}

void loop() {
  // put your main code here, to run repeatedly:
  // int leftSpd = 70;

  ECE3_read_IR(sensorValues);  // Read sensor values into array with size 8

  PDController();

  // analogWrite(left_pwm_pin,leftSpd);

  //

  //  ECE3_read_IR(sensorValues);
}
