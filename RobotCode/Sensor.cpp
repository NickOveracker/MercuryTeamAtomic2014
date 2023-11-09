#include <Arduino.h>

class Sensor
{
  private:
    byte echoPin;
    byte trigPin;
    long reading;
    
  public:
    Sensor(byte _echoPin, byte _trigPin)
    {
      echoPin = _echoPin;
      trigPin = _trigPin;
      pinMode(trigPin, OUTPUT);
    }
    
    byte readSensor()
    {
      digitalWrite(trigPin, LOW);
      delayMicroseconds(2);
      digitalWrite(trigPin, HIGH);
      delayMicroseconds(10);
      digitalWrite(trigPin, LOW);
      // Get the measurement in centimeters.
      // Time out after 60 milliseconds, which should be more than enough time.
      // I mean, seriously, why is sound so freaking slow?
      // Cap the reading at 255.
      reading = pulseIn(echoPin, HIGH, 30000) / 58;
      return reading <= 255 ? (byte) reading : 255;
    }
};
