# Balancierroboter

Dieses Projekt ist der Quellcode für einen Balancierroboter mit einem Ball auf einer Kippplatte.

![](images/Balancierroboter1.jpg)

### Verwendete Hardware

* ESP32-WROOM-32A
* 2x Servomotor
* 3x Potentiometer
* 3x Taster
* Joystick
* Touchscreen

![](images/Neue_Platine_beschriftet.jpeg)

## Servomotorbeschaltung

### Knopf

Steuere die Position eines [Hobby Servo's](http://en.wikipedia.org/wiki/Servo_motor#RC_servos) mit deinem Arduino und einem Potentiometer.

Dieses Beispiel verwendet die arduino "Servo"-Bibliothek.

### Zusammenschaltung

Servo motors have three wires: power, ground, and signal. The power wire is typically red, and should be connected to the 5V pin on the Arduino board. The ground wire is typically black or brown and should be connected to a ground pin on the board. The signal pin is typically yellow or orange and should be connected to pin 9 on the board.

The potentiometer should be wired so that its two outer pins are connected to power (+5V) and ground, and its middle pin is connected to analog input 0 on the board.

![Servosteuerung Kabelanordnung](images/Servosteuerung_Kabelanordnung.jpg)

(Die Schaltbilder wurden mit [Fritzing.org](http://fritzing.org/) erstellt.)


### Schaltplan

![Servosteuerung Schaltplan](images/Servosteuerung_Schaltplan.jpg)
