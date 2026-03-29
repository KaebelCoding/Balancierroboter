// INITIALISIERUNG --------------------------------------------------------------------------------
#include <ESP32ServoController.h>
#include "AdvancedPID.h"

using namespace MDO::ESP32ServoController;

// === uC-Pin definition ===
#define PinButtonPower 15
#define PinButtonRegelbetrieb 2
#define PinButtonJoysticksteuerung 0
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
float joystickAngleTranslation = 0.65;  // Übersetzungsverhältnis zwischen Joystick- und Servowinkel

ServoController ServoX, ServoY;  // create servo object to control a servo
float servoAngleX;
float servoAngleY; 
float servoOffsetX = 85.0; //Servopositionen bei ungeneigter Ebene
float servoOffsetY = 90.0;

int touchX = 0, touchY = 0, touchXOld = -1, touchYOld = -1;  
uint32_t touchXTimer = 0, touchYTimer;
float posX = 0, posY = 0;

float Kb = 0;
float Kp, KpMax = 0.2;  // ursprünglicher Wert = 1
float Ki, KiMax = 0.002; // ursprünglich = 0
float Kd, KdMax = 0.1;  // ursprünglich = 0.5 

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
  PIDX.setDeadband(1);  // Threshold for the error
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
  Serial.begin(115200); // Baud Rate = 115200, weil Joystick sonst verzögert reagiert 
}

// BETRIEB ----------------------------------------------------------------------------------------
void loop() {
  // Warum wird der Joystick ausgelesen, wenn der Joystickmodus vlt. gar nicht aktiv ist?
  joystickReadingX = analogRead(PinJoystickX); // (brauche ich diese Zwischenspeicher-Variable wirklich?)
  joystickReadingY = analogRead(PinJoystickY); // (brauche ich diese Zwischenspeicher-Variable wirklich?)
  measureTouchscreenXAxis();
  measureTouchscreenYAxis();
  Serial.print ("\n");

  joystickAngleX = (joystickOffsetX - joystickReadingX) / 30.00; // (der Joystick reagiert aktuell noch sehr grob)
  joystickAngleY = (joystickReadingY - joystickOffsetY) / 30.00; // (vlt. kann man ihn hier feiner einstellen)
  
  Serial.print("Joysitck X-Winkel:\t");
  Serial.print(joystickAngleX);
  Serial.print("\tJoysitck Y-Winkel:\t");
  Serial.print(joystickAngleY);
  Serial.print("\n");

  switch (mode) {
    case 0:
      Serial.print ("Modus 1 - StandBy/An schalten \n");
      servoAngleX = 0;
      servoAngleY = 0;
      break;
    case 1:
      Serial.print ("Modus 2 - Regelbetrieb \n");
      // Warum werden Joystickwerte im Regelbetrieb verwendet? Sollte der Regelbetrieb nicht komplett entkoppelt vom Joystick sein?
      servoAngleX = PIDX.run(posX, joystickAngleX * 3); // output = myPID.run(input, setpoint);
      servoAngleY = PIDY.run(posY, joystickAngleY * 3); // neue Zielwert-Variablen erstellen und probieren 
                                                        // posX und posY: sind Abstände zum Zielwert (Plattenmitte)
                                                        // joystickAngle: ist mögliche Verschiebung des Zielpunktes
      // Serial.print("X-Position:\t");
      // Serial.print(posX);
      // Serial.print("Joystick X-Winkel:\t");
      // Serial.print(joystickAngleX);
      // Serial.print("Servo X-Winkel:\t");
      // Serial.print(servoAngleX);
      // Serial.print("\n");
      break;
    case 2:
      Serial.print ("Modus 3 - Joysticksteuerung \n");
      servoAngleX = joystickAngleX * joystickAngleTranslation;
      servoAngleY = joystickAngleY * joystickAngleTranslation;
      break;
  }
  Serial.print("\n");

  delay(20); // in ms
  // min. und max. Beschränkung der Servowinkel
  servoAngleX = constrain(servoAngleX, -45, 45);
  servoAngleY = constrain(servoAngleY, -45, 45);

  ServoX.moveTo(servoAngleX + servoOffsetX, 0, false);
  ServoY.moveTo(-servoAngleY + servoOffsetY, 0, false);

  // 10 mal pro Sekunde wird:
  if (every100ms > millis()) {
    every100ms = millis() + 100;
    
  // aktuelle Regelwerte berechnen (Potis werden ausgelesen)
    Kp = (4096 -analogRead(PinPotiP)) * KpMax / 4096;    // evtl Möglichkeit finden feste Regelparameter einzustellen, z.B: Taste gedrückt halten 
    Ki = (4096 -analogRead(PinPotiI)) * KiMax / 4096;    // Poti Abfrage in Case Regelbetrieb verschieben 
    Kd = (4096 -analogRead(PinPotiD)) * KdMax / 4096;

    PIDX.setTunings(Kp, Ki, Kd);
    PIDY.setTunings(Kp, Ki, Kd);

    // Interface-Buttons auslesen und Modus setzen
    if (not digitalRead(PinButtonPower)) {      // hier wird immer mit "not" gearbeitet, wäre es möglich hier auch ohne "not" zu arbeiten?
      mode = 0;
      Serial.println("Modus: An/Aus \n");
    }
    if (not digitalRead(PinButtonRegelbetrieb)) {
      mode = 1;
      Serial.println("Modus: Regelbetrieb \n");
    }
    if (not digitalRead(PinButtonJoysticksteuerung)) {
      mode = 2;
      Serial.println("Modus: Joysticksteuerung \n");
    }
  }
}

// FUNKTIONEN ------------------------------------------------------------------------------------
void measureTouchscreenXAxis() {
  // Stelle die Touchscreen-Pins ein
  pinMode(PinY1, INPUT);      // Y-Pins
  pinMode(PinY2, INPUT);
  digitalWrite(PinY2, LOW);
  pinMode(PinX1, OUTPUT);     // X-Pins
  digitalWrite(PinX1, HIGH);
  delay(1); // in ms
  pinMode(PinX2, OUTPUT);
  digitalWrite(PinX2, LOW);
  delay(2); // in ms

  // Lese den Touchscreen-X-Wert aus
  touchX = analogRead(PinY1);
  Serial.print("Touchscreen x-Position:\t");
  Serial.print(touchX);

  if (abs(touchX - touchXOld) < 100 || millis() > touchXTimer) {
    touchXTimer = millis() + 100;
    touchXOld = touchX;
    posX = (touchX - 1877) * 0.121535; // die empirischen Werte hier sollten Namen bekommen, damit man weiß was wozu gehört (ggf. auch für Anpassungen wichtig)
    //Serial.print("0");
  }
  // Debugging
  Serial.print("\t Calculated x-Position:\t");
  Serial.print(posX);
  Serial.print("\n");
}

void measureTouchscreenYAxis() {
  // Stelle die Touchscreen-Pins ein
  pinMode(PinX1, INPUT);     // X-Pins
  pinMode(PinX2, INPUT);
  digitalWrite(PinX2, LOW);
  pinMode(PinY1, OUTPUT);    // Y-Pins
  digitalWrite(PinY1, HIGH);
  delay(1); // in ms
  pinMode(PinY2, OUTPUT);
  digitalWrite(PinY2, LOW);
  delay(2); // in ms

  // Lese den Touchscreen-Y-Wert aus
  touchY = analogRead(PinX1);
  //Serial.print("\t");
  Serial.print("Touchscreen y-Position:\t");
  Serial.print(touchY);
  
  if (abs(touchY - touchYOld) < 100 || millis() > touchYTimer) {
    touchYTimer = millis() + 100;
    touchYOld = touchY;
    posY = (touchY - 1992) * 0.10280; // die empirischen Werte hier sollten Namen bekommen, damit man weiß was wozu gehört (ggf. auch für Anpassungen wichtig)
    //Serial.print("0");
  }
  // Debugging
  Serial.print("\t Calculated y-Position:\t");
  Serial.print(posY);
  Serial.print("\n");
}
