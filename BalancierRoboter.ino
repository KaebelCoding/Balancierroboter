// INITIALISIERUNG --------------------------------------------------------------------------------
#include <ESP32ServoController.h>
#include "AdvancedPID.h"

using namespace MDO::ESP32ServoController;

// === uC-Pin definition ===
#define PinButtonPower 0    
#define PinButtonRegelbetrieb 2
#define PinButtonJoysticksteuerung 15  
#define PinPotiP 16
#define PinPotiI 17
#define PinPotiD 5
#define PinJoystickX 39 // Inputs - Joystick
#define PinJoystickY 36
#define PinX1 26        // Inputs - Touchscreensensor
#define PinX2 33        // (is x1 the signal and x2 the ground/vcc?? -> maybe better naming possible)
#define PinY1 25
#define PinY2 32
#define PinServoX 14    // Outputs - Servo
#define PinServoY 12

uint8_t mode = 0;
long every100ms = 100;

int joystickReadingX;
int joystickReadingY;
int joystickOffsetX = 1840;  //Joystickposition bei keiner Auslenkung
int joystickOffsetY = 1821;
float joystickAngleX;
float joystickAngleY;

ServoController ServoX, ServoY;  // create servo object to control a servo
float servoAngleX;
float servoAngleY; 
float servoOffsetX = 85.0; //Servopositionen bei ungeneigter Ebene
float servoOffsetY = 90.0;

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
  
  pinMode(PinButtonPower, INPUT_PULLUP);
  pinMode(PinButtonRegelbetrieb, INPUT_PULLUP);
  pinMode(PinButtonJoysticksteuerung, INPUT_PULLUP);
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
  Serial.begin(115200); // Baud Rate, größer als 9600, weil Joystick sonst verzögert reagiert 
}

// BETRIEB ----------------------------------------------------------------------------------------

void loop() {
  
  joystickReadingX = analogRead(PinJoystickX); // (brauche ich diese Zwischenspeicher-Variable wirklich?)
  joystickReadingY = analogRead(PinJoystickY); // (brauche ich diese Zwischenspeicher-Variable wirklich?)
  measureTouchscreenXAxis();
  measureTouchscreenYAxis();

  joystickAngleX = (joystickOffsetX - joystickReadingX) / 30.00; // (der Joystick reagiert aktuell noch sehr grob)
  joystickAngleY = (joystickReadingY - joystickOffsetY) / 30.00; // (vlt. kann man ihn hier feiner einstellen)
  //Serial.print("X ");
  //Serial.print(joystickAngleX);
  //Serial.print("\t Y ");
  //Serial.print(joystickAngleY);

  switch (mode) {
    case 0:  // Power An/Aus
      Serial.print (" Modus 3 - Aus/An schalten");
      servoAngleX = 0;
      servoAngleY = 0;
      break;
    case 1:  // Regelbetrieb
      servoAngleX = PIDX.run(posX, joystickAngleX * 3);  //  output = myPID.run(input, setpoint);
      servoAngleY = PIDY.run(posY, joystickAngleY * 3);
      Serial.print (" Modus 2 - Regelbetrieb");
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
    case 2:  // Joysticksteuerung
      servoAngleX = joystickAngleX;
      servoAngleY = joystickAngleY;
      Serial.print (" Modus 1 - Joysticksteuerung");
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
    if (not digitalRead(PinButtonPower)) {
      mode = 0;
      Serial.println("Modus: An/Aus");
    }
    if (not digitalRead(PinButtonRegelbetrieb)) {
      mode = 1;
      Serial.println("Modus: Regelbetrieb");
    }
    if (not digitalRead(PinButtonJoysticksteuerung)) {
      mode = 2;
      Serial.println("Modus: Joysticksteuerung");
    }
  }
}

// FUNKTIONEN ------------------------------------------------------------------------------------

<<<<<<< HEAD:BallancierRoboter/BallancierRoboter.ino
void measureTouchscreenXAxis() {          
=======
void measureTouchscreenXAxis() {
>>>>>>> 144d95379d23c52c01f88083fa5469763bfc12d2:BallancierRoboter.ino
  pinMode(PinY1, INPUT);
  pinMode(PinY2, INPUT);
  digitalWrite(PinY2, LOW);<

  pinMode(PinX1, OUTPUT);
  digitalWrite(PinX1, HIGH);
  delay(1); // in ms
  pinMode(PinX2, OUTPUT);
  digitalWrite(PinX2, LOW);
  delay(2); // in ms
  touchX = analogRead(PinY1);
  Serial.print(touchX);
  Serial.print("\t");

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

void measureTouchscreenYAxis() {
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
  //Serial.print("\t");
  Serial.print(touchY);
  Serial.print("\t");
  
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