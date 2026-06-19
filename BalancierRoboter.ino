// BIBLIOTHEKEN --------------------------------------------------------------------------------
#include <ESP32ServoController.h>
#include "AdvancedPID.h"

using namespace MDO::ESP32ServoController;

// === uC-Pin-Belegung ===
#define PinButtonPower 16
#define PinButtonRegelbetrieb 17
#define PinButtonJoysticksteuerung 5
#define PinLEDPower 22
#define PinLEDRegelbetrieb 23
#define PinLEDJoysticksteuerung 21
#define PinPotiP 4
#define PinPotiI 2
#define PinPotiD 18
#define PinJoystickX 39                           // Inputs - Joystick
#define PinJoystickY 36
#define PinX1 26                                  // Touchscreensensor (evtl. erklärendere Variablennamen möglich)
#define PinX2 33
#define PinY1 25
#define PinY2 32
#define PinServoX 14                              // Output-Pins - Servos
#define PinServoY 12

// === Systemvariablen ===
uint8_t mode = 0;                                 // Startet in StandBy (Modus 1 von 3)
long TimerIntervall = 20;                        // Frequenzgebende Variable T = 1/f
long RecentControl = 0;
long RecentPotiReading = 0; 

// === Prozessvariablen ===
int joystickOffsetX = 1840;                       // Joystickposition bei keiner Auslenkung
int joystickOffsetY = 1821;                       
float joystickAngleX;
float joystickAngleY;
float joystickAngleUseabilityScaling = 0.65;      // Übersetzungsverhältnis zwischen Joystick- und Servowinkel
float TranslationADCValueToJoystickAngle = 30.0;

ServoController ServoX, ServoY;                   // erzeuge Servo-Objekt um den Servomotor zu steuern
float servoAngleX;
float servoAngleY;
float servoOffsetX = 85.0;                        // Servopositionen bei ungeneigter Ebene 
float servoOffsetY = 90.0;

int touchX = 0, touchY = 0, touchXOld = -1, touchYOld = -1;
uint32_t touchXTimer = 0, touchYTimer = 0;
float posX = 0, posY = 0;
int TouchScreenXOffsetToMiddle = 1877;
int TouchScreenYOffsetToMiddle = 1992;

float RegelungszielX;
float RegelungszielY;
float ServoSprungbegrenzung = 10;                 // Idee: raduisabhängig erhöhen/verringern
static float ServoAngleXRecent = 0;
static float ServoAngleYRecent = 0;

bool Datenausgabe = 1;                            // Funktion für die Datenausgabe aktiv/inaktiv schaltbar
unsigned long MessStartZeit = 0;

float Kb = 0;
float Kp, KpMax = 0.24;
float Ki, KiMax = 0.015;
float Kd, KdMax = 0.125;

AdvancedPID PIDX(Kp, Ki, Kd, Kb);       
AdvancedPID PIDY(Kp, Ki, Kd, Kb);       

void setup() {
  pinMode(PinButtonPower, INPUT_PULLUP);
  pinMode(PinButtonRegelbetrieb, INPUT_PULLUP);
  pinMode(PinButtonJoysticksteuerung, INPUT_PULLUP);
  pinMode(PinLEDPower, OUTPUT);
  pinMode(PinLEDRegelbetrieb, OUTPUT);
  pinMode(PinLEDJoysticksteuerung, OUTPUT);
  pinMode(PinPotiP, INPUT);
  pinMode(PinPotiI, INPUT);
  pinMode(PinPotiD, INPUT);

  PIDX.setOutputLimits(-45, 45);
  PIDY.setOutputLimits(-45, 45);
  PIDX.setDeadband(1);  // Threshold for the error
  PIDY.setDeadband(1);
  PIDX.setDerivativeFilter(0.8);
  PIDY.setDerivativeFilter(0.8);
 
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

  digitalWrite(PinLEDPower, LOW);
  digitalWrite(PinLEDRegelbetrieb, LOW);
  digitalWrite(PinLEDJoysticksteuerung, LOW);
  delay(300);
  digitalWrite(PinLEDPower, HIGH);
  digitalWrite(PinLEDRegelbetrieb, HIGH);
  digitalWrite(PinLEDJoysticksteuerung, HIGH);
  delay(300);
  digitalWrite(PinLEDPower, HIGH);
  digitalWrite(PinLEDRegelbetrieb, LOW);
  digitalWrite(PinLEDJoysticksteuerung, LOW);
  delay(300);

  Serial.begin(115200); // Baud Rate = 115200, weil Joystick sonst verzögert reagiert 
}

// BETRIEB ----------------------------------------------------------------------------------------
void loop() {
    // Interface-Buttons auslesen und Modus setzen
    if (not digitalRead(PinButtonPower)) {
      mode = 0;
      Serial.print("Modus 1 - StandBy\n");

      digitalWrite(PinLEDPower, HIGH);
      digitalWrite(PinLEDRegelbetrieb, LOW);
      digitalWrite(PinLEDJoysticksteuerung, LOW);
    }
    if (not digitalRead(PinButtonRegelbetrieb)) {
      mode = 1;
      //Serial.print ("Modus 2 - Regelbetrieb\n");
      MessStartZeit = millis();

      digitalWrite(PinLEDPower, LOW);
      digitalWrite(PinLEDRegelbetrieb, HIGH);
      digitalWrite(PinLEDJoysticksteuerung, LOW);
    }
    if (not digitalRead(PinButtonJoysticksteuerung)) {
      mode = 2;
      Serial.print ("Modus 3 - Joysticksteuerung\n");

      digitalWrite(PinLEDPower, LOW);
      digitalWrite(PinLEDRegelbetrieb, LOW);
      digitalWrite(PinLEDJoysticksteuerung, HIGH);
      }

  switch (mode) {
    case 0:
      servoAngleX = 0; // keine gerade Platte, sondern nur
      servoAngleY = 0; // Nullwinkel der Servos (-> beliebig)
      break;
    case 1:
      
      if (millis() - RecentControl >= TimerIntervall) {
        measureTouchscreenXAxis();
        measureTouchscreenYAxis();
        measureJoystickAngles();
        
        // aktuelle Reglerparameter berechnen (Potis werden ausgelesen)
        if (millis() - RecentPotiReading > 500) {
          Kp = (4096 -analogRead(PinPotiP)) * KpMax / 4096;   // Mögliche Erweiterung: feste Regelparameter einstellen, z.B: Taste gedrückt halten
          Ki = (4096 -analogRead(PinPotiI)) * KiMax / 4096;
          Kd = (4096 -analogRead(PinPotiD)) * KdMax / 4096;   
          PIDX.setTunings(Kp, Ki, Kd);
          PIDY.setTunings(Kp, Ki, Kd);
          RecentPotiReading = millis();
        }

        RegelungszielX = 0;                     // Verschieben/Ändern des Zielpunktes möglich; z.B. mit: joystickAngleX * 3; joystickAngleY * 3;
        RegelungszielY = 0;  

        servoAngleX = PIDX.run(posX, RegelungszielX); // output = myPID.run(input, setpoint) bzw. abweichung = .run(istwert, führungsgröße)
        servoAngleY = PIDY.run(posY, RegelungszielY);
        ServoStabilisierung();                        // glättet zu starke Sprünge
        RecentControl = millis();
      }  
      if(Datenausgabe = 1) {

        Datenabruf();
      }
      break;
    case 2:
      measureJoystickAngles();
      servoAngleX = joystickAngleX * joystickAngleUseabilityScaling;
      servoAngleY = -joystickAngleY * joystickAngleUseabilityScaling;
      break;
  }
  
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

  if (abs(touchX - touchXOld) <= 100 || millis() >= touchXTimer) {  
      
    touchXTimer = millis() + 100;
    posX = (touchX - TouchScreenXOffsetToMiddle) * 0.121535; // die empirischen Werte hier sollten Namen bekommen, damit man weiß was wozu gehört (ggf. auch für Anpassungen wichtig)
    touchXOld = touchX; 
  }
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

  if (abs(touchY - touchYOld) <= 100 || millis() >= touchYTimer) {    
    
    touchYTimer = millis() + 100;
    posY = (touchY - TouchScreenYOffsetToMiddle) * 0.10280; // die empirischen Werte hier sollten Namen bekommen, damit man weiß was wozu gehört (ggf. auch für Anpassungen wichtig)
    touchYOld = touchY; 
  }
}

void measureJoystickAngles() {
  joystickAngleX = (joystickOffsetX - analogRead(PinJoystickX)) / TranslationADCValueToJoystickAngle;
  joystickAngleY = (joystickOffsetY - analogRead(PinJoystickY)) / TranslationADCValueToJoystickAngle;
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

void PunktZuKomma(float Wert) {

  String Komma_Wert = String(Wert);
  Komma_Wert.replace(".", ",");
  Serial.print(Komma_Wert);
  Serial.print("\t");
}

void Datenabruf() {

  Serial.print(millis() - MessStartZeit);
  Serial.print("\t");

  PunktZuKomma(posX);
    
  PunktZuKomma(posY);
  
  PunktZuKomma(RegelungszielX);
  
  PunktZuKomma(RegelungszielY);
 
  PunktZuKomma(servoAngleX);
  
  PunktZuKomma(servoAngleY);
  
  PunktZuKomma(Kp);
  
  PunktZuKomma(Ki);
  
  PunktZuKomma(Kd);
  Serial.print("\n");
}