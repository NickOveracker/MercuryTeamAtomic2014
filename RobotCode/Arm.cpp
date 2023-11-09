#include <Arduino.h>
#include <Servo.h>

class Arm
{
  private:
    boolean magnet, armRight, armLeft, liftArm, lowerArm;
    byte actuatorExtendPin, actuatorRetractPin, actuatorKillPin, magnetPin;
    byte servoPos;
    Servo servo;
    
  public:
    Arm(byte _actuatorExtendPin, byte _actuatorRetractPin, byte _actuatorKillPin, byte _magnetPin, Servo _servo)
    {
      actuatorExtendPin = _actuatorExtendPin;
      actuatorRetractPin = _actuatorRetractPin;
      actuatorKillPin = _actuatorKillPin;
      magnetPin = _magnetPin;
      servo = _servo;
      
      pinMode(actuatorExtendPin, OUTPUT);
      pinMode(actuatorRetractPin, OUTPUT);
      pinMode(magnetPin, OUTPUT);
      //pinMode(actuatorKillPin, INPUT);
      
      servoPos = 80;
      servo.write(servoPos);
    }
    
    void setMotion(byte armControlByte)
    {
      // Handle the input.
      magnet   = armControlByte & 128 ? true : false;
      armRight = armControlByte &  64 ? true : false;
      armLeft  = armControlByte &  32 ? true : false;
      liftArm  = armControlByte &  16 ? true : false;
      lowerArm = armControlByte &   8 ? true : false;
 
      // SET THE SERVO
      // Do not attempt to rotate both left and right simultaneously.
      if(armRight && armLeft)
      {
        armRight = armLeft = false;
      }
      if(armLeft)
      {
        turnArm(-10);
      }
      else if(armRight)
      {
        turnArm(10);
      }
      
      // SET THE ACTUATOR
      // Do not attempt to raise and lower simultaneously.
      if(liftArm && lowerArm)
      {	
        liftArm = lowerArm = false; 
      }
      // Only raise the arm if the kill switch is not triggered.
      if(liftArm /* && !digitalRead(actuatorKillPin)*/)
      {
        raise();
      }
      // Allow the arm to be lowered regardless of the kill switch.
      else if(lowerArm)
      {
        lower();
      }
      else
      {
        stopVertical();
      }
      
      // SET THE MAGNET
      digitalWrite(magnetPin, magnet);
    } 
    
    void turnArm(byte amount)
    {
      servoPos -= amount;
      if(servoPos == 0)
      {
        servoPos = 10;
      }
      else if(servoPos == 180)
      {
        servoPos = 170;
      }
      servo.write(servoPos);
    }
    
    void resetArm()
    {
      servoPos = 90;
      servo.write(servoPos);
    }
      
    void raise()
    {  
      digitalWrite(actuatorExtendPin, LOW); 
      digitalWrite(actuatorRetractPin, HIGH); 
    }

    void lower()
    {
      digitalWrite(actuatorRetractPin, LOW);
      digitalWrite(actuatorExtendPin, HIGH); 
    }

    void stopVertical()
    {
      digitalWrite(actuatorExtendPin, LOW);
      digitalWrite(actuatorRetractPin, LOW);
    }
    
    byte getServoPos()
    {
      return servoPos;
    }
};
