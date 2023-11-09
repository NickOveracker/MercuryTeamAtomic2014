//Due to the nature of this project, my friend Nick has forced/advised 
//me to comment on my code. I believe code should be as hard to read
//as it is to write. He feels differently. Hopefully this robot runs
//at all so to help I will add the following to its source code. 
//~YOU ARE A WINNER. YOU ARE NAMED (insert name here). BRING THE GOLD HOME!~
// 0.02 John P. / Nick O.

#include "Arm.cpp"
#include "Motor.cpp"
#include "Sensor.cpp"
#include <Servo.h>

const byte 
  // Every command should start with this
  SYNC_BYTE = 0xAA,
  // Autostop cutoff
  TOO_CLOSE_FRONT = 15,
  TOO_CLOSE_SIDES = 12,
  // I/O Pins
  ARM_KILL_PIN                = 2,
  LEFT_MOTOR_PWM_PIN          = 3,
  LEFT_MOTOR_A_PIN            = 4,
  LEFT_MOTOR_B_PIN            = 5,
  HORN_PIN                    = 6,
  ARM_EXTEND_PIN              = 7,
  ARM_RETRACT_PIN             = 8,
  ARM_SERVO_PIN               = 9,  // Must be PWM.
  RIGHT_MOTOR_B_PIN           = 10, // Swapped 10 and 11 from original.
  RIGHT_MOTOR_PWM_PIN         = 11, // Servo.h disables PWM on 10.
  RIGHT_MOTOR_A_PIN           = 12,
  ARM_MAGNET_PIN              = 13,
  BACK_LEFT_SENSOR_TRIG_PIN   = A0,
  BACK_RIGHT_SENSOR_TRIG_PIN  = A1,
  FRONT_LEFT_SENSOR_TRIG_PIN  = A2,
  FRONT_RIGHT_SENSOR_TRIG_PIN = A3,
  FRONT_SENSOR_TRIG_PIN       = A4,
  ALL_SENSORS_ECHO_PIN        = A5;
  
const short
  // Send output after this many milliseconds
  OUTPUT_INTERVAL = 300,
  // Expect input to be available after AT MOST this many milliseconds
  // We are being very generous with our patience here. 300ms is the
  // actual interval.
  INPUT_TIMEOUT = 1200;
           
// Peripherals
Motor rightMotor(
  RIGHT_MOTOR_A_PIN,
  RIGHT_MOTOR_B_PIN,
  RIGHT_MOTOR_PWM_PIN
);

Motor leftMotor(
  LEFT_MOTOR_A_PIN,
  LEFT_MOTOR_B_PIN,
  LEFT_MOTOR_PWM_PIN
);
                 
Sensor
  backRightSensor(
    ALL_SENSORS_ECHO_PIN,
    BACK_RIGHT_SENSOR_TRIG_PIN
  ),
  backLeftSensor(
    ALL_SENSORS_ECHO_PIN,
    BACK_LEFT_SENSOR_TRIG_PIN
  ),
  frontRightSensor(
    ALL_SENSORS_ECHO_PIN,
    FRONT_RIGHT_SENSOR_TRIG_PIN
  ),
  frontLeftSensor(
    ALL_SENSORS_ECHO_PIN,
    FRONT_LEFT_SENSOR_TRIG_PIN
  ),
  frontSensor(
    ALL_SENSORS_ECHO_PIN,
    FRONT_SENSOR_TRIG_PIN
  );
  
Servo servo;
               
Arm arm(
  ARM_EXTEND_PIN,
  ARM_RETRACT_PIN,
  ARM_KILL_PIN,
  ARM_MAGNET_PIN,
  servo
);
        
byte
  // These hold bytes received over serial.
  syncByte,
  leftMotorByte,
  rightMotorByte,
  controlsByte,
  checksumByte,
  // Sensor input
  backLeftSensorData,
  backRightSensorData,
  frontLeftSensorData,
  frontRightSensorData,
  frontSensorData,
  outputChecksum;
  
// We'll store output values here until they are sent out.
byte outputBytes[8];
  
boolean autoStopMode = true,
        autoPilotMode = false,
        signalLost = false,
        isTooClose = false;

long lastInputTime = 0, lastOutputTime = 0; // For measuring time intervals.

void setup()
{
  Serial.begin(9600);
  pinMode(HORN_PIN, OUTPUT);
  pinMode(ALL_SENSORS_ECHO_PIN, INPUT);
  servo.attach(ARM_SERVO_PIN);
  outputBytes[0] = (byte) 0xBB; // output sync byte.
  SOS();
  signalLost = false;
} 

void loop()
{
  //each byte represents a byte from the internet input  
  if(Serial.available() >= 5)
  {
    // Synchronize. Just keep reading in until we have the sync byte,
    // so long as we have enough bytes waiting in the buffer.
    while(Serial.available() >= 5 && (syncByte = Serial.read()) != SYNC_BYTE)
    {
      /* No op */
    }

    //read all the things!
    if(syncByte == SYNC_BYTE)
    {
      leftMotorByte = Serial.read();
      rightMotorByte = Serial.read();
      controlsByte = Serial.read();
      checksumByte = Serial.read();
      
      // Verify the checksum
      if((leftMotorByte ^ rightMotorByte ^ controlsByte) == checksumByte)
      {
        handleInput();
        signalLost = false;
        lastInputTime = millis();
      }
    }
  }
  else if(millis() - lastInputTime > INPUT_TIMEOUT)
  {
    leftMotor.stopMoving();
    rightMotor.stopMoving();
    arm.stopVertical();
    signalLost = true;
    SOS();
  }
  
  if(millis() - lastOutputTime > OUTPUT_INTERVAL && !signalLost)
  {
    sendSensorData();
    lastOutputTime = millis();
  }
 
}

void handleInput()
{
  arm.setMotion(controlsByte);
  
  // Autostop
  if(!autoPilotMode && autoStopMode && isTooClose)
  {
    leftMotorByte = rightMotorByte = 0;
  }
  // Autopilot
  if(autoPilotMode)
  {
    followLeftWall();
  }
  else if(leftMotorByte || rightMotorByte)
  {
    leftMotor.setMotion(leftMotorByte);
    rightMotor.setMotion(rightMotorByte);
  }
  else
  {
    leftMotor.stopMoving();
    rightMotor.stopMoving();
  }
  
  // Drive mode stuff
  autoStopMode  = controlsByte & 4;
  autoPilotMode = controlsByte & 2;
  
  // Meep meep!
  digitalWrite(HORN_PIN, controlsByte & 1);
  
  return;
}

void sendSensorData()
{ 
  // The sync byte was set in setup().
  outputBytes[1] = backLeftSensorData = backLeftSensor.readSensor();
  outputBytes[2] = backRightSensorData = backRightSensor.readSensor();
  outputBytes[3] = frontLeftSensorData = frontLeftSensor.readSensor();
  outputBytes[4] = frontRightSensorData = frontRightSensor.readSensor();
  outputBytes[5] = frontSensorData = frontSensor.readSensor();
  outputBytes[6] = arm.getServoPos();
  outputBytes[7] = frontLeftSensorData ^ frontSensorData; // checksum
  
  /*
  outputBytes[1] = 1;
  outputBytes[2] = 2;
  outputBytes[3] = 3;
  outputBytes[4] = 4;
  outputBytes[5] = 5;
  outputBytes[6] = 6;
  outputBytes[7] = 3 ^ 5;
  */
  
  
  Serial.write(outputBytes, 8);
  Serial.flush();
  
  // Set isTooClose if necessary.
  isTooClose = (min4(
                     backLeftSensorData,
                     backRightSensorData,
                     frontLeftSensorData,
                     frontRightSensorData
                   ) <= TOO_CLOSE_SIDES)
                   ||
                   (frontSensorData <= TOO_CLOSE_FRONT);
}

void SOS()
{
  analogWrite(HORN_PIN, 64);
  delay(50);
  digitalWrite(HORN_PIN, LOW);
  delay(50);
  analogWrite(HORN_PIN, 64);
  delay(50);
  digitalWrite(HORN_PIN, LOW);
  delay(50);
  analogWrite(HORN_PIN, 64);
  delay(50);
  digitalWrite(HORN_PIN, LOW);
  delay(150);
  analogWrite(HORN_PIN, 64);
  delay(150);
  digitalWrite(HORN_PIN, LOW);
  delay(50);
  analogWrite(HORN_PIN, 64);
  delay(150);
  digitalWrite(HORN_PIN, LOW);
  delay(50);
  analogWrite(HORN_PIN, 64);
  delay(150);
  digitalWrite(HORN_PIN, LOW);
  delay(150);
  analogWrite(HORN_PIN, 64);
  delay(50);
  digitalWrite(HORN_PIN, LOW);
  delay(50);
  analogWrite(HORN_PIN, 64);
  delay(50);
  digitalWrite(HORN_PIN, LOW);
  delay(50);
  analogWrite(HORN_PIN, 64);
  delay(50);
  digitalWrite(HORN_PIN, LOW);
  delay(350);
}

// Gets the minimum of 5 inputs.
byte min4(byte a, byte b, byte c, byte d)
{
  a = min(a, b);
  a = min(a, c);
  a = min(a, d);
  return a;
}

// Follows the... do I really have to explain this?
void followLeftWall()
{
  // Don't hit front wall!
  if(frontSensorData <= 25)
  {
    leftMotor.stopMoving();
    rightMotor.stopMoving();
    return;
  }
  
  leftMotor.setMotion(80); // Speed 80/127 forward.
  
  // See if the front sensor is less than 9cm from the wall.
  if(frontLeftSensorData < 9)
  {
    // Decrease the right motor speed to go to the right.
    rightMotor.setMotion(80 - (9 - (frontLeftSensorData)<<1));
  }
  // See if the front left sensor is more than 15cm from the wall.
  else if(frontLeftSensorData > 15)
  {
    // Increase the right motor speed to go to the left.
    rightMotor.setMotion(min(80 + ((frontLeftSensorData - 15)<<1), 127));
  }
  else
  {
    rightMotor.setMotion(80);
  }
}
