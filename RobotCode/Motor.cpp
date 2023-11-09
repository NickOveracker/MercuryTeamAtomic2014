#include <Arduino.h>

class Motor
{
  private:
    byte pinNumA, pinNumB, speedPin;
   
  public:
    Motor(byte _pinNumA, byte _pinNumB, byte _speedPin)
    {
       pinNumA = _pinNumA;
       pinNumB = _pinNumB;
       speedPin = _speedPin;
       pinMode(pinNumA, OUTPUT);
       pinMode(pinNumB, OUTPUT);
       pinMode(speedPin, OUTPUT);
    }
  
    void setMotion(byte input)
    {
       // Move backward if reverse bit is true
       if(input & 128){
        digitalWrite(pinNumA, HIGH);
        digitalWrite(pinNumB, LOW);
       }
       else
       {
        digitalWrite(pinNumA, LOW);
        digitalWrite(pinNumB, HIGH);
       }
 
       //This will remove the direction bit
        analogWrite(speedPin,(input << 1)); 
    }
  
    void stopMoving()
    {
      analogWrite(speedPin, 0);
      digitalWrite(pinNumA, HIGH);
      digitalWrite(pinNumB, HIGH);
    }
};
