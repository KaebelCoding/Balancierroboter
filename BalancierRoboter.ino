// INITIALISIERUNG --------------------------------------------------------------------------------
#include <ESP32ServoController.h>
#include "AdvancedPID.h"

using namespace MDO::ESP32ServoController;

// === uC-Pin-Belegung ===
#define PinButtonPower 16
#define PinButtonRegelbetrieb 17
#define PinButtonJoysticksteuerung 5
#define PinPotiP 4
#define PinPotiI 2
#define PinPotiD 0
#define PinJoystickX 39                 // Inputs - Joystick
#define PinJoystickY 36
#define PinX1 26                        // Inputs - Touchscreensensor
#define PinX2 33                        // Evtl. erklärendere Variablennamen möglich
#define PinY1 25
#define PinY2 32
#define PinServoX 14                    // Outputs - Servos
#define PinServoY 12

uint8_t mode = 0;                       // Startet in StandBy (Modus 1 von 3)
long every100ms = 100;

int joystickOffsetX = 1840;             // Joystickposition bei keiner Auslenkung
int joystickOffsetY = 1821;             // genauere Bezeichnung hilfreich (Offset von was?)
float joystickAngleX;
float joystickAngleY;
float joystickAngleUseabilityScaling = 0.65;  // Übersetzungsverhältnis zwischen Joystick- und Servowinkel
float TranslationADCValueToJoystickAngle = 30.0;

ServoController ServoX, ServoY;         // erzeuge Servo-Objekt um den Servomotor zu steuern
float servoAngleX;
float servoAngleY;
float servoOffsetX = 85.0;              // Servopositionen bei ungeneigter Ebene (bessere Bezeichnung als Offset möglich? Vlt. ZeroPosition? Oder EvenPlateAngle oder Balanced Plate Angle?)
float servoOffsetY = 90.0;

int touchX = 0, touchY = 0, touchXOld = -1, touchYOld = -1;
uint32_t touchXTimer = 0, touchYTimer = 0;
float posX = 0, posY = 0;
int TouchScreenXOffsetToMiddle = 1877;
int TouchScreenYOffsetToMiddle = 1992;

float RegelungszielX;
float RegelungszielY;
float ServoSprungbegrenzung = 15; 
static float ServoAngleXRecent = 0;
static float ServoAngleYRecent = 0;

float Kb = 0;
float Kp, KpMax = 0.24;                
float Ki, KiMax = 0.005;                
float Kd, KdMax = 0.125;                  

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

  ServoFactoryDecorator oFactoryDecorator(oTimerChannelFactory);  // ServoFactoryDecorator definiert Servo-Frequenz und weiteres
  if (!ServoX.begin(oFactoryDecorator, PinServoX)) {
    Serial.println("Initiierung von X-Servomotor fehlgeschlagen.\n");
    return;
  }
  if (!ServoY.begin(oFactoryDecorator, PinServoY)) {
    Serial.println("Initiierung von Y-Servomotor fehlgeschlagen.\n");
    return;
  }
  Serial.begin(115200); // Baud Rate = 115200, weil Joystick sonst verzögert reagiert 
}

// BETRIEB ----------------------------------------------------------------------------------------
void loop() {
    // Interface-Buttons auslesen und Modus setzen
    // Refactoring-Vorschlag: hier wird immer mit "not" gearbeitet, wäre es möglich hier auch ohne "not" zu arbeiten? Was gibt denn die digitalRead-Fkt. für einen Rückgabewert?
    
    if (not digitalRead(PinButtonPower))              {mode = 0;}
    if (not digitalRead(PinButtonRegelbetrieb))       {mode = 1;}
    if (not digitalRead(PinButtonJoysticksteuerung))  {mode = 2;}

  switch (mode) {
    case 0:
      Serial.print ("Modus 1 - StandBy\n");
      servoAngleX = 0; // keine gerade Platte, sondern nur
      servoAngleY = 0; // Nullwinkel der Servos (-> beliebig)

      // Serial.print("P-Wert:");
      // Serial.print(Kp);
      // Serial.print("\tI-Wert:");
      // Serial.print(Ki);
      // Serial.print("\tD-Wert:");
      // Serial.print(Kd);
      // Serial.print("\n");
      // Serial.print("PotiP-Wert:");
      // Serial.print(analogRead(PinPotiP));
      // Serial.print("\tPotiI-Wert:");
      // Serial.print(analogRead(PinPotiI));
      // Serial.print("\tPotiD-Wert:");
      // Serial.print(analogRead(PinPotiD));
      // Serial.print("\n");

      break;
    case 1:
      Serial.print ("Modus 2 - Regelbetrieb\n");
      measureTouchscreenXAxis();
      measureTouchscreenYAxis();
      measureJoystickAngles();
      
      // aktuelle Reglerparameter berechnen (Potis werden ausgelesen)
      if (every100ms < millis()) {                          // 10 mal pro Sekunde
        every100ms = millis() + 100;       
        Kp = (4096 -analogRead(PinPotiP)) * KpMax / 4096;   // evtl Möglichkeit finden feste Regelparameter einzustellen, z.B: Taste gedrückt halten 
        Ki = 0.001;   // ursprünglich: (4096 -analogRead(PinPotiI)) * KiMax / 4096
        Kd = (4096 -analogRead(PinPotiI)) * KdMax / 4096;   // Poti fehlt -> Ki fester Wert und Kd auf Poti ursprünglich: (4096 -analogRead(PinPotiD)) * KdMax / 4096
      }

      PIDX.setTunings(Kp, Ki, Kd);
      PIDY.setTunings(Kp, Ki, Kd);
      PIDX.setDerivativeFilter(0.8);
      PIDY.setDerivativeFilter(0.8);
      PIDX.setIntegralZone(10);
      PIDY.setIntegralZone(10);
      RegelungszielX = joystickAngleX * 3;  // Joystickwerte zum möglichen Verschieben des Zielpunktes
      RegelungszielY = joystickAngleY * 3;  

      servoAngleX = PIDX.run(posX, RegelungszielX); // output = myPID.run(input, setpoint) bzw. abweichung = .run(istwert, führungsgröße)
      servoAngleY = PIDY.run(posY, RegelungszielY);
      ServoStabilisierung();

      Serial.print("X-Position: ");
      Serial.print(posX);
      Serial.print("\tServo X-Winkel:\t");
      Serial.print(servoAngleX);
      Serial.print("\tServo Y-Winkel: ");
      Serial.print(servoAngleY);
      Serial.print("\n");
      Serial.print("X-Ziel:\t");
      Serial.print(RegelungszielX);
      Serial.print("\n");
      Serial.print("Y-Ziel:\t");
      Serial.print(RegelungszielY);
      Serial.print("\n");
      break;
    case 2:
      Serial.print ("Modus 3 - Joysticksteuerung\n");
      measureJoystickAngles();

      servoAngleX = joystickAngleX * joystickAngleUseabilityScaling;
      servoAngleY = joystickAngleY * joystickAngleUseabilityScaling;

      Serial.print("Servo X-Winkel:\t");
      Serial.print(servoAngleX);
      Serial.print("Servo Y-Winkel:\t");
      Serial.print(servoAngleY);
      Serial.print("\n");
      
      // Serial.print("Joysitck Offset X:");
      // Serial.print(analogRead(PinJoystickX));
      // Serial.print("\tJoysitck Offset Y:");
      // Serial.print(analogRead(PinJoystickY));
      // Serial.print("\n");

      break;
  }
  Serial.print("\n");

  delay(1); // in ms
  
  servoAngleX = constrain(servoAngleX, -45, 45); // min. und max. Beschränkung der Servowinkel
  servoAngleY = constrain(servoAngleY, -45, 45);

  ServoX.moveTo(servoAngleX + servoOffsetX, 0, false);
  ServoY.moveTo(-servoAngleY + servoOffsetY, 0, false);
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

  if (abs(touchX - touchXOld) <= 100 || millis() >= touchXTimer) {  
      
    touchXTimer = millis() + 100;
    posX = (touchX - TouchScreenXOffsetToMiddle) * 0.121535; // die empirischen Werte hier sollten Namen bekommen, damit man weiß was wozu gehört (ggf. auch für Anpassungen wichtig)
    touchXOld = touchX; // mit Filter: (touchXOld - TouchScreenXOffsetToMiddle) * 0.121535 * 0.9 + (touchX - TouchScreenXOffsetToMiddle) * 0.121535 * 0.1
    Serial.print("\ttouchX: ");
    Serial.print(touchX);
    Serial.print("\ttouchXOld: ");
    Serial.print(touchXOld);
    Serial.print("\ttouchXTimer: ");
    Serial.print(touchXTimer);
    Serial.print("\n");
  }
  // Debugging
  Serial.print("Calculated x-Position: ");
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
  Serial.print("Touchscreen y-Position:\t");
  Serial.print(touchY);
  
  if (!abs(touchY - touchYOld) <= 100 || millis() >= touchYTimer) {    
    
    touchYTimer = millis() + 100;
    posY = (touchY - TouchScreenYOffsetToMiddle) * 0.10280; // die empirischen Werte hier sollten Namen bekommen, damit man weiß was wozu gehört (ggf. auch für Anpassungen wichtig)
    touchYOld = touchY; // mit Filter: (touchYOld - TouchScreenYOffsetToMiddle) * 0.10280 * 0.8 + (touchY - TouchScreenYOffsetToMiddle) * 0.10280 * 0.2
    Serial.print("\ttouchY: ");
    Serial.print(touchY);
    Serial.print("\ttouchYOld: ");
    Serial.print(touchYOld);
    Serial.print("\ttouchYTimer: ");
    Serial.print(touchYTimer);
    Serial.print("\n");
  }
  // Debugging
  Serial.print("Calculated y-Position:\t");
  Serial.print(posY);
  Serial.print("\n");
}

void measureJoystickAngles() {
  joystickAngleX = (joystickOffsetX - analogRead(PinJoystickX)) / TranslationADCValueToJoystickAngle;
  joystickAngleY = (joystickOffsetY - analogRead(PinJoystickY)) / TranslationADCValueToJoystickAngle;
  Serial.print("Joysitck X-Winkel: ");
  Serial.print(joystickAngleX);
  Serial.print("\tJoysitck Y-Winkel: ");
  Serial.print(joystickAngleY);
  Serial.print("\n");
}

void ServoStabilisierung() {

  float AngleDiffX =  servoAngleX - ServoAngleXRecent;
  float AngleDiffY =  servoAngleY - ServoAngleYRecent;

  if (AngleDiffX > ServoSprungbegrenzung) AngleDiffX = ServoSprungbegrenzung;
  if (AngleDiffX < -ServoSprungbegrenzung) AngleDiffX = -ServoSprungbegrenzung;
  if (AngleDiffY > ServoSprungbegrenzung) AngleDiffY = ServoSprungbegrenzung;
  if (AngleDiffY < -ServoSprungbegrenzung) AngleDiffY = -ServoSprungbegrenzung;

  ServoAngleXRecent = ServoAngleXRecent + AngleDiffX;
  ServoAngleYRecent = ServoAngleYRecent + AngleDiffY;
  servoAngleX = ServoAngleXRecent;
  servoAngleY = ServoAngleYRecent;
}
