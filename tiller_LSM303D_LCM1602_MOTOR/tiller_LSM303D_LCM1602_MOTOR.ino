
#include <Wire.h>
#include <LSM303.h>
#include <LiquidCrystal_I2C.h> 
#include <ArduinoSort.h>

LSM303 compass;

//
// LCD BEGIN VARS
//

// The LCD constructor - address shown is 0x27 - may or may not be correct for yours
// Also based on YWRobot LCM1602 IIC V1
LiquidCrystal_I2C lcd(0x27, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);  

//
// LCD END VARS
//

//
// MOTOR BEGIN VARS
//

// MOTOR pinouts
//The following pins are in use only if the DC/Stepper noted is in use:
//Digital pin 11: DC Motor #1 / Stepper #1 (activation/speed control)
//Digital pin 3: DC Motor #2 / Stepper #1 (activation/speed control)
//Digital pin 5: DC Motor #3 / Stepper #2 (activation/speed control)
//Digital pin 6: DC Motor #4 / Stepper #2 (activation/speed control)
//
//The following pins are in use if any DC/steppers are used
//Digital pin 4, 7, 8 and 12 are used to drive the DC/Stepper motors via the 74HC595 serial-to-parallel latch

// Arduino pins for the shift register
#define MOTORLATCH 12
#define MOTORCLK 4
#define MOTORENABLE 7
#define MOTORDATA 8

// 8-bit bus after the 74HC595 shift register 
// (not Arduino pins)
// These are used to set the direction of the bridge driver.
#define MOTOR1_A 2
#define MOTOR1_B 3
#define MOTOR2_A 1
#define MOTOR2_B 4
#define MOTOR3_A 5
#define MOTOR3_B 7
#define MOTOR4_A 0
#define MOTOR4_B 6

// Arduino pins for the PWM signals.
#define MOTOR1_PWM 11
#define MOTOR2_PWM 3
#define MOTOR3_PWM 6
#define MOTOR4_PWM 5
#define SERVO1_PWM 10
#define SERVO2_PWM 9

// Codes for the motor function.
#define FORWARD 1
#define BACKWARD 2
#define BRAKE 3
#define RELEASE 4

// MOTOR END VARS

const int leftButtonPin = 2;
const int rightButtonPin = 3;
const int toggleManualModeButtonPin = 5;
const int delayBetweenHeadingReadsInNumberOfButtonDetectPulses = 10;
const int deviationAllowed = 2; // degrees on each side of current
const int targetAdjustmentInDegrees = 1; // how much to adjust target by for each button pulse
const int durationOfMajorAdjustmentInPulseIntervals = 3;
const int durationOfMinorAdjustmentInPulseIntervals = 1;
const int deviationThresholdForMinorAdjustment = 10;
const int buttonDetectPulseInterval = 50;  // how long to wait between checking for button presses

int target = 0;
int heading = 0;
int lastHeading = 0;
int lastTarget = 0;
boolean manualMode = false;
int compassCorrection = 160;


void setup() {

  // See https://www.arduino.cc/en/Tutorial/DigitalPins
  // wire buttons to GND and dig pin, then value LOW will indicate button pressed
  pinMode(leftButtonPin, INPUT_PULLUP);
  pinMode(rightButtonPin, INPUT_PULLUP);
  pinMode(toggleManualModeButtonPin, INPUT_PULLUP);
  
  Serial.begin(9600);

  // Leonardo: wait for serial port to connect
  while (!Serial) 
    {
    }

  Serial.println("setup()");
  Wire.begin();
  Serial.println("after Wire.begin()");
  compass.init();
  Serial.println("after compass.init()");
  compass.enableDefault();
  Serial.println("after compass.enableDefault()");
  
  /*
  Calibration values; the default values of +/-32767 for each axis
  lead to an assumed magnetometer bias of 0. Use the Calibrate example
  program to determine appropriate values for your particular unit.
  */
  compass.m_min = (LSM303::vector<int16_t>){-32767, -32767, -32767};
  compass.m_max = (LSM303::vector<int16_t>){+32767, +32767, +32767};

  // LCD
  lcd.begin(16,2); // sixteen characters across - 2 lines
  Serial.println("after lcd.begin(16,2)");
  lcd.backlight();
  //lcd.noBacklight();
  // first character - 1st line
  lcd.print("Welcome aboard!");

  // initialize target heading to current heading
  heading = readCompassHeadingWithCorrection();
  target = heading;
  lastHeading = heading;
  lastTarget = target;
  
  lcd.setCursor(0,1);
  lcd.print("Heading: ");
  lcd.print(target);
  delay(3000);
}

void loop() {

  Serial.println("loop() start");
  heading = readNextHeading();

  if (manualMode) {
    displayInManualMode();  

    // Don't do anything, just wait for buttons to be pressed
    delayAndHandleButtonsPressed(delayBetweenHeadingReadsInNumberOfButtonDetectPulses, false);
  } else {

    // we are in auto mode
    displayInAutoMode();  

    // calculate heading adjustment needed to reach target
    int adjustment = calculateAdjustment(target, heading);
    boolean rotateLeft = false;
    if (adjustment < 0) {
      rotateLeft = true;
    }

    // how far off are we?  do we need just a minor adjustment?
    boolean minorAdjustment = false;
    if (abs(adjustment) < deviationThresholdForMinorAdjustment) {
      minorAdjustment = true;
    }

    // are we already rotating in the right direction?
    boolean rotatingTheRightWay = false;
    if (target == lastTarget) { // target hasn't changed. (tossing out last rotation direction if we just changed target.)
      int lastAdjustment = calculateAdjustment(target, lastHeading);
      if (abs(adjustment) < abs(lastAdjustment)) { // the adjustment is smaller, so we must be rotating the right way already
        rotatingTheRightWay = true;
      }
    }

    // perform auto adjustment if needed
    if (rotatingTheRightWay) {
      Serial.println(" => hold, already rotating in the right direction.");
    } else if (abs(target - heading) <= deviationAllowed) {
      Serial.print(" => hold, difference is less than ");
      Serial.print(deviationAllowed);
      Serial.println(" degrees.");
    } else {
      Serial.print(" => ");
      performAdjustment(rotateLeft, minorAdjustment);
    }
    
    // delay before next heading read / auto adjustment, but watch out for
    // buttons being pressed in the meantime
    delayAndHandleButtonsPressed(delayBetweenHeadingReadsInNumberOfButtonDetectPulses, true);
  }

  // Store the last heading and target after delays
  lastHeading = heading;
  lastTarget = target;
}

void performAdjustment(boolean rotateLeft, boolean minorAdjustment) {
  if (rotateLeft) {
      Serial.print("rotating left.. ");
      motor(1, FORWARD, 255);
    } else {
      Serial.print("rotating right.. ");
      motor(1, BACKWARD, 255);
    }

    // delay to allow time for actuator to run
    if (minorAdjustment) {
      Serial.print("minor adjustment.. ");
      delayAndHandleButtonsPressed(durationOfMinorAdjustmentInPulseIntervals, true);
    } else {
      Serial.print("major adjustment.. ");
      delayAndHandleButtonsPressed(durationOfMajorAdjustmentInPulseIntervals, true);
    }
    
    Serial.println("done rotating.");
    motor(1, RELEASE, 0); // stop pulse
}

void delayAndHandleButtonsPressed(int delayDurationInPulses, boolean ignoreManualAdjustments) {
  for (int i = 0; i < delayDurationInPulses; i++) {

    // left / to port
    int leftButtonState = digitalRead(leftButtonPin);
    if (leftButtonState == LOW) {
      if (manualMode == true) {        
        if (!ignoreManualAdjustments) {
          Serial.println("manual mode: left button pressed, adjusting");
          performAdjustment(true, true);  
        } else {
          Serial.println("manual mode: ignoring left button pressed, we are already adjusting");                  
        }
      } else {
        Serial.println("auto mode: left button pressed, decreasing target heading");
        // auto mode -- decrease target heading to turn left / port
        target -= targetAdjustmentInDegrees;
        if (target < 0) {
          target = target + 360;
        } 
      }
    }
      
    // right / to starboard
    int rightButtonState = digitalRead(rightButtonPin);
    if (rightButtonState == LOW) {
      if (manualMode == true) {
        if (!ignoreManualAdjustments) {
          Serial.println("manual mode: right button pressed, adjusting");
          performAdjustment(false, true);
        } else {
          Serial.println("manual mode: ignoring right button pressed, we are already adjusting");                  
        }
      } else {
        Serial.println("auto mode: right button pressed, increasing target heading");
        // auto mode -- increase target heading to turn right / starboard
        target += targetAdjustmentInDegrees; 
        if (target >= 360) {
          target = 0;
        }
      }      
    } 
    
    // toggle manual/auto
    int toggleManualModeState = digitalRead(toggleManualModeButtonPin);
    if (toggleManualModeState == LOW) {      

      // wait for the toggle button to be released before proceeding
      // (so that we avoid any misunderstandings related to how long the button is pressed)
      while (toggleManualModeState == LOW) {        
        toggleManualModeState = digitalRead(toggleManualModeButtonPin);  
        delay(10);
      }

      if (manualMode == true) {

        // turn on auto mode
        Serial.println("manual mode button was pressed then released: turning on auto mode");
        manualMode = false;        
        heading = heading; // start with target heading equal to current heading
      } else {

        // turn on manual mode
        Serial.println("manual mode button was pressed then released: turning on manual mode");
        manualMode = true;  
      }      
    } 

    // delay between taking action on each button detect
//    Serial.print("delay ");
//    Serial.println(buttonDetectPulseInterval);
    delay(buttonDetectPulseInterval);
  }
}

void displayInAutoMode() {
  lcd.home();
  lcd.clear();
  lcd.print("Heading: ");
  if (heading < 10) {
    lcd.print("  ");
  } else if (heading < 100) {
    lcd.print(" ");
  }
  lcd.print(heading);
  lcd.print(" ");  
  lcd.print(degreesToDirection(heading));
  lcd.setCursor(0,1);
  lcd.print("Target : ");
  if (target < 10) {
    lcd.print("  ");
  } else if (target < 100) {
    lcd.print(" ");
  }
  lcd.print(target);
  lcd.print(" ");  
  lcd.print(degreesToDirection(target));

  int delta = abs(heading - target);
  Serial.print("Heading: ");
  Serial.print(heading);
  Serial.print(", Target: ");
  Serial.print(target);
  Serial.print(", delta: ");
  Serial.print(delta);
  Serial.print(", Last heading: ");
  Serial.print(lastHeading);
}

void displayInManualMode() {
  lcd.home();
  lcd.clear();
  lcd.print("Heading: ");
  if (heading < 10) {
    lcd.print("  ");
  } else if (heading < 100) {
    lcd.print(" ");
  }
  lcd.print(heading);
  lcd.print(" ");  
  lcd.print(degreesToDirection(heading));

  lcd.setCursor(0,1);
  lcd.print("Manual Mode");
  
  Serial.print("Manual Mode");
  Serial.print("Heading: ");
  Serial.println(heading);
  Serial.print("Last heading: ");
  Serial.println(lastHeading);
}

// ---------------------------------
// motor
//
// Select the motor (1-4), the command, 
// and the speed (0-255).
// The commands are: FORWARD, BACKWARD, BRAKE, RELEASE.
//
void motor(int nMotor, int command, int speed)
{
  int motorA, motorB;

  if (nMotor >= 1 && nMotor <= 4)
  {  
    switch (nMotor)
    {
    case 1:
      motorA   = MOTOR1_A;
      motorB   = MOTOR1_B;
      break;
    case 2:
      motorA   = MOTOR2_A;
      motorB   = MOTOR2_B;
      break;
    case 3:
      motorA   = MOTOR3_A;
      motorB   = MOTOR3_B;
      break;
    case 4:
      motorA   = MOTOR4_A;
      motorB   = MOTOR4_B;
      break;
    default:
      break;
    }

    switch (command)
    {
    case FORWARD:
      motor_output (motorA, HIGH, speed);
      motor_output (motorB, LOW, -1);     // -1: no PWM set
      break;
    case BACKWARD:
      motor_output (motorA, LOW, speed);
      motor_output (motorB, HIGH, -1);    // -1: no PWM set
      break;
    case BRAKE:
      // The AdaFruit library didn't implement a brake.
      // The L293D motor driver ic doesn't have a good
      // brake anyway.
      // It uses transistors inside, and not mosfets.
      // Some use a software break, by using a short
      // reverse voltage.
      // This brake will try to brake, by enabling 
      // the output and by pulling both outputs to ground.
      // But it isn't a good break.
      motor_output (motorA, LOW, 255); // 255: fully on.
      motor_output (motorB, LOW, -1);  // -1: no PWM set
      break;
    case RELEASE:
      motor_output (motorA, LOW, 0);  // 0: output floating.
      motor_output (motorB, LOW, -1); // -1: no PWM set
      break;
    default:
      break;
    }
  }
}


// ---------------------------------
// motor_output
//
// The function motor_ouput uses the motor driver to
// drive normal outputs like lights, relays, solenoids, 
// DC motors (but not in reverse).
//
// It is also used as an internal helper function 
// for the motor() function.
//
// The high_low variable should be set 'HIGH' 
// to drive lights, etc.
// It can be set 'LOW', to switch it off, 
// but also a 'speed' of 0 will switch it off.
//
// The 'speed' sets the PWM for 0...255, and is for 
// both pins of the motor output.
//   For example, if motor 3 side 'A' is used to for a
//   dimmed light at 50% (speed is 128), also the 
//   motor 3 side 'B' output will be dimmed for 50%.
// Set to 0 for completelty off (high impedance).
// Set to 255 for fully on.
// Special settings for the PWM speed:
//    Set to -1 for not setting the PWM at all.
//
void motor_output (int output, int high_low, int speed)
{
  int motorPWM;

  switch (output)
  {
  case MOTOR1_A:
  case MOTOR1_B:
    motorPWM = MOTOR1_PWM;
    break;
  case MOTOR2_A:
  case MOTOR2_B:
    motorPWM = MOTOR2_PWM;
    break;
  case MOTOR3_A:
  case MOTOR3_B:
    motorPWM = MOTOR3_PWM;
    break;
  case MOTOR4_A:
  case MOTOR4_B:
    motorPWM = MOTOR4_PWM;
    break;
  default:
    // Use speed as error flag, -3333 = invalid output.
    speed = -3333;
    break;
  }

  if (speed != -3333)
  {
    // Set the direction with the shift register 
    // on the MotorShield, even if the speed = -1.
    // In that case the direction will be set, but
    // not the PWM.
    shiftWrite(output, high_low);

    // set PWM only if it is valid
    if (speed >= 0 && speed <= 255)    
    {
      analogWrite(motorPWM, speed);
    }
  }
}

// ---------------------------------
// shiftWrite
//
// The parameters are just like digitalWrite().
//
// The output is the pin 0...7 (the pin behind 
// the shift register).
// The second parameter is HIGH or LOW.
//
// There is no initialization function.
// Initialization is automatically done at the first
// time it is used.
//
void shiftWrite(int output, int high_low)
{
  static int latch_copy;
  static int shift_register_initialized = false;

  // Do the initialization on the fly, 
  // at the first time it is used.
  if (!shift_register_initialized)
  {
    // Set pins for shift register to output
    pinMode(MOTORLATCH, OUTPUT);
    pinMode(MOTORENABLE, OUTPUT);
    pinMode(MOTORDATA, OUTPUT);
    pinMode(MOTORCLK, OUTPUT);

    // Set pins for shift register to default value (low);
    digitalWrite(MOTORDATA, LOW);
    digitalWrite(MOTORLATCH, LOW);
    digitalWrite(MOTORCLK, LOW);
    // Enable the shift register, set Enable pin Low.
    digitalWrite(MOTORENABLE, LOW);

    // start with all outputs (of the shift register) low
    latch_copy = 0;

    shift_register_initialized = true;
  }

  // The defines HIGH and LOW are 1 and 0.
  // So this is valid.
  bitWrite(latch_copy, output, high_low);

  // Use the default Arduino 'shiftOut()' function to
  // shift the bits with the MOTORCLK as clock pulse.
  // The 74HC595 shiftregister wants the MSB first.
  // After that, generate a latch pulse with MOTORLATCH.
  shiftOut(MOTORDATA, MOTORCLK, MSBFIRST, latch_copy);
  delayMicroseconds(5);    // For safety, not really needed.
  digitalWrite(MOTORLATCH, HIGH);
  delayMicroseconds(5);    // For safety, not really needed.
  digitalWrite(MOTORLATCH, LOW);
}

String degreesToDirection(int degrees) {
  if (degrees <= 22) return "N";
  if (degrees > 22 && degrees <= 67) return "NE";
  if (degrees > 67 && degrees <= 112) return "E";
  if (degrees > 112 && degrees <= 157) return "SE";
  if (degrees > 157 && degrees <= 202) return "S";
  if (degrees > 202 && degrees <= 247) return "SW";
  if (degrees > 247 && degrees <= 292) return "W";
  if (degrees > 292 && degrees <= 337) return "NW";
  if (degrees > 337) return "N";
  return "N/A";
}

// calculate the heading adjustment needed to line up with target
int calculateAdjustment(int target, int heading) {
  int delta = target - heading;
  int deltaAbs = abs(delta);

  if (deltaAbs > 180) {
    int degreesFrom180 = abs(180 - deltaAbs);
    if (target >= 180) {
      return -1 * (180 - degreesFrom180);
    }
    return (180 - degreesFrom180);
  } 
  
  // deltaAbs < 180
  return delta;  
}

int correct(int h, int correction) {
  int correctedHeading = h + correction;
  if (correctedHeading >= 360) {
    correctedHeading = correctedHeading - 360;
  } else if (correctedHeading < 0) {
    correctedHeading = 360 - correctedHeading;
  }
  return correctedHeading;
}

int readCompassHeadingWithCorrection() {
  compass.read();

  /*
  When given no arguments, the heading() function returns the angular
  difference in the horizontal plane between a default vector and
  north, in degrees.
  
  The default vector is chosen by the library to point along the
  surface of the PCB, in the direction of the top of the text on the
  silkscreen. This is the +X axis on the Pololu LSM303D carrier and
  the -Y axis on the Pololu LSM303DLHC, LSM303DLM, and LSM303DLH
  carriers.
  
  To use a different vector as a reference, use the version of heading()
  that takes a vector argument; for example, use
  
    compass.heading((LSM303::vector<int>){0, 0, 1});
  
  to use the +Z axis as a reference.
  
  */  
  int h = compass.heading();
  return correct(h, compassCorrection);
}

int readNextHeading() {

  long start = millis();

  int headings[9];
  for (int i = 0; i < 9; i++) {
    headings[i] = readCompassHeadingWithCorrection();
  }

  sortArray(headings, 9);
  return headings[4];
}
