#include <ESP32ServoController.h>
#include "AdvancedPID.h"

using namespace MDO::ESP32ServoController;

// Input uC-Pins
#define Button1 15
#define Button2 4
#define Button3 0
#define PinPotiP 32
#define PinPotiI 35
#define PinPotiD 34
#define PinJoystickX 39
#define PinJoystickY 36
#define X1 13
#define X2 14
#define Y1 12
#define Y2 27
// Output uC-Pins
#define PinServoX 26
#define PinServoY 25
// #define X1 12
// #define X2 27
// #define Y1 13
// #define Y2 14

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
  pinMode(Button1, INPUT_PULLUP);
  pinMode(Button2, INPUT_PULLUP);
  pinMode(Button3, INPUT_PULLUP);
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
    Serial.println("  failed to init the servo..");
    return;
  }
  if (!ServoY.begin(oFactoryDecorator, PinServoY)) {  //3rd parameter is the default angle to start from: 90 degrees in this case
    Serial.println("  failed to init the servo..");
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
    if (not digitalRead(Button1)) {
      mode = 0;
      //Serial.println("Mode 0");
    }
    if (not digitalRead(Button2)) {
      mode = 1;
      //Serial.println("Mode 1");
    }
    if (not digitalRead(Button3)) {
      mode = 2;
      //Serial.println("Mode 2");
    }
  }
}

void measureX() {
  pinMode(Y1, INPUT);
  pinMode(Y2, INPUT);
  digitalWrite(Y2, LOW);

  pinMode(X1, OUTPUT);
  digitalWrite(X1, HIGH);
  delay(1); // in ms
  pinMode(X2, OUTPUT);
  digitalWrite(X2, LOW);
  delay(2); // in ms
  touchX = analogRead(Y1);
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
  pinMode(X1, INPUT);
  pinMode(X2, INPUT);
  digitalWrite(X2, LOW);
  pinMode(Y1, OUTPUT);
  digitalWrite(Y1, HIGH);
  delay(1); // in ms
  pinMode(Y2, OUTPUT);
  digitalWrite(Y2, LOW);
  delay(2); // in ms

  touchY = analogRead(X1);
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
