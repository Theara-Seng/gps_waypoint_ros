// motor.cpp

#include "motor.h"


//Normal Motor class
Motor::Motor(uint8_t pinIN1, uint8_t pinIN2, uint8_t mainSpeedPin, uint8_t secondarySpeedPin): _pinIN1(pinIN1), _pinIN2(pinIN2), _mainSpeedPin(mainSpeedPin), _secondarySpeedPin(secondarySpeedPin),enc(mainSpeedPin, secondarySpeedPin){
  // constructeur de la classe Motor
}


 void Motor::initPins() {      
  pinMode(_pinIN1, OUTPUT);
  pinMode(_pinIN2, OUTPUT);
  }

  
void Motor::commandMotor(int command) {  
  command = max(-255,min(255,command));
  if (command > 0) {
    digitalWrite(_pinIN2, LOW);
    analogWrite(_pinIN1, command);
    }
  else if (command < 0) {
    digitalWrite(_pinIN1, LOW);
    analogWrite(_pinIN2, -command);
    } 
  else {
    digitalWrite(_pinIN2, LOW);
    digitalWrite(_pinIN1, LOW);
    } 
  }
