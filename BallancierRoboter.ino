#include <ESP32ServoController.h>
#include "AdvancedPID.h"

using namespace MDO::ESP32ServoController;

// === uC-Pin definition ===
#define PinButton1 15   // Inputs - Interface
#define PinButton2 4
#define PinButton3 0
#define PinPotiP 32
#define PinPotiI 35
#define PinPotiD 34
#define PinJoystickX 39 // Inputs - Joystick
#define PinJoystickY 36
#define PinX1 13        // Inputs - Touchscreen
#define PinX2 14
#define PinY1 12
#define PinY2 27
#define PinServoX 26    // Outputs - Servo
#define PinServoY 25
// #define PinX1 12
// #define PinX2 27
// #define PinY1 13
// #define PinY2 14

uint8_t mode = 0;
long every100ms = 100;

int joystickReadingX;
int joystickReadingY;
int joystickOffsetX = 1840;
int joystickOffsetY = 1821;
float joystickAngleX;
float joystickAngleY;

ServoController ServoX, ServoY;  // create servo object to control a servo
float servoAngleX, servoAngleY, servoOffsetX = 85.0, servoOffsetY = 90.0;

int touchX = 0, touchY = 0, touchXOld = -1, touchYOld = -1;
uint32_t touchXTimer = 0, touchYTimer;
float posX = 0, posY = 0;

float Kb = 0;
float Kp, KpMax = 1;
float Ki, KiMax = 0.0;
float Kd, KdMax = 0.50;

AdvancedPID PIDX(Kp, Ki, Kd, Kb);
AdvancedPID PIDY(Kp, Ki, Kd, Kb);

void setup() {
  pinMode(PinButton1, INPUT_PULLUP);
  pinMode(PinButton2, INPUT_PULLUP);
  pinMode(PinButton3, INPUT_PULLUP);
  pinMode(PinPotiP, INPUT);
  pinMode(PinPotiI, INPUT);
  pinMode(PinPotiD, INPUT);

  PIDX.setOutputLimits(-45, 45);
  PIDY.setOutputLimits(-45, 45);
  PIDX.setDeadband(1);
  PIDY.setDeadband(1);

  Esp32LedcRegistry::instance()->begin(LEDC_CONFIG_ESP32);
  BestAvailableFactory oTimerChannelFactory;

  ServoFactoryDecorator oFactoryDecorator(oTimerChannelFactory);  //let this ServoFactoryDecorator define the servo frequency to use and such
  if (!ServoX.begin(oFactoryDecorator, PinServoX)) {              //3rd parameter is the default angle to start from: 90 degrees in this case
    Serial.println("  failed to init the x-servo..");
    return;
  }
  if (!ServoY.begin(oFactoryDecorator, PinServoY)) {  //3rd parameter is the default angle to start from: 90 degrees in this case
    Serial.println("  failed to init the y-servo..");
    return;
  }
  Serial.begin(115200);
}

void loop() {
  joystickReadingX = analogRead(PinJoystickX);
  joystickReadingY = analogRead(PinJoystickY);
  measureX();
  measureY();

  joystickAngleX = (joystickOffsetX - joystickReadingX) / 30.00;
  joystickAngleY = (joystickReadingY - joystickOffsetY) / 30.00;
  //Serial.print("X ");
  //Serial.print(joystickAngleX);
  //Serial.print("\t Y ");
  //Serial.print(joystickAngleY);

  switch (mode) {
    case 0:  //Manual Controll
      servoAngleX = joystickAngleX;
      servoAngleY = joystickAngleY;
      Serial.print (" Mode1");
      break;
    case 1:
      servoAngleX = PIDX.run(posX, joystickAngleX * 3);  //  output = myPID.run(input, setpoint);
      servoAngleY = PIDY.run(posY, joystickAngleY * 3);
      Serial.print (" Mode2");
      //Serial.print(millis());
      /*
      Serial.print("\t");
      Serial.print(posX);
      Serial.print("\t");
      Serial.print(joystickAngleX);
      Serial.print("\t");
      Serial.print(servoAngleX);
      */
      break;
    case 2:  //off
      Serial.print (" Mode3");
      servoAngleX = 0;
      servoAngleY = 0;
      break;
  }
  Serial.println();

  //Serial.print(servoAngleX);
  //Serial.print("\t");
  //Serial.println(servoAngleY);

  delay(20); // in ms

  servoAngleX = constrain(servoAngleX, -45, 45);
  servoAngleY = constrain(servoAngleY, -45, 45);

  ServoX.moveTo(servoAngleX + servoOffsetX, 0, false);
  ServoY.moveTo(-servoAngleY + servoOffsetY, 0, false);

  if (every100ms > millis()) {
    every100ms = millis() + 100;
    
    Kp = (4096 -analogRead(PinPotiP)) * KpMax / 4096;
    Ki = (4096 -analogRead(PinPotiI)) * KiMax / 4096;
    Kd = (4096 -analogRead(PinPotiD)) * KdMax / 4096;

    PIDX.setTunings(Kp, Ki, Kd);
    PIDY.setTunings(Kp, Ki, Kd);

    // Auslesen des Interfaces
    if (not digitalRead(PinButton1)) {
      mode = 0;
      //Serial.println("Mode 0");
    }
    if (not digitalRead(PinButton2)) {
      mode = 1;
      //Serial.println("Mode 1");
    }
    if (not digitalRead(PinButton3)) {
      mode = 2;
      //Serial.println("Mode 2");
    }
  }
}

void measureX() {
  pinMode(PinY1, INPUT);
  pinMode(PinY2, INPUT);
  digitalWrite(PinY2, LOW);

  pinMode(PinX1, OUTPUT);
  digitalWrite(PinX1, HIGH);
  delay(1); // in ms
  pinMode(PinX2, OUTPUT);
  digitalWrite(PinX2, LOW);
  delay(2); // in ms
  touchX = analogRead(PinY1);
  //Serial.print(touchX);
  //Serial.print("\t");

  if (abs(touchX - touchXOld) > 100 && millis() < touchXTimer) {
    //Serial.print("400");
  } else {   //Normal
    touchXTimer = millis() + 100;
    touchXOld = touchX;
    posX = (touchX - 1877) * 0.121535;
    //Serial.print("0");
  }
  
  Serial.print("x-Position:\t");
  Serial.print(posX);
}

void measureY() {
  pinMode(PinX1, INPUT);
  pinMode(PinX2, INPUT);
  digitalWrite(PinX2, LOW);
  pinMode(PinY1, OUTPUT);
  digitalWrite(PinY1, HIGH);
  delay(1); // in ms
  pinMode(PinY2, OUTPUT);
  digitalWrite(PinY2, LOW);
  delay(2); // in ms

  touchY = analogRead(PinX1);
  /*Serial.print("\t");
  Serial.print(touchY);
  Serial.print("\t");
  */
  if (abs(touchY - touchYOld) > 100 && millis() < touchYTimer) {
    //Serial.print("500");
  } else {
    touchYTimer = millis() + 100;
    touchYOld = touchY;
    posY = (touchY - 1992) * 0.10280;
    //Serial.print("0");
  }
  
  Serial.print("y-Position:\t");
  Serial.print(posY);
}
