/**
 * The RobotState is a data structure intended simply to hold the current
 * state of the robot.
 *
 * Nick Overacker
 * Team Atomic
 * Mercury Robotics
 */

public class RobotState
{
   // These constants are used to describe the motion of the
   // actuator and servo.
   public static final byte STATIONARY =  0;
   public static final byte OUTWARD    =  1;
   public static final byte INWARD     = -1;

   // Motion
   private byte leftMotorSpeed  = 0;
   private byte rightMotorSpeed = 0;
   private byte servoMotion     = STATIONARY;
   private byte actuatorMotion  = STATIONARY;
   private boolean eMagnetIsOn  = false;

   // Received data
   private int backLeftSensorData   = 0;
   private int backRightSensorData  = 0;
   private int frontLeftSensorData  = 0;
   private int frontRightSensorData = 0;
   private int frontSensorData      = 0;
   private int servoPosition        = 0;

   // Other stuff
   private byte    controlMode    = 0;
   private double  curveVal       = 0;
   private boolean bridgeModeIsOn = false;
   private boolean autoStopIsOn   = false;
   private boolean autoPilotIsOn  = false;
   private boolean hornIsOn       = false;
   private boolean isConnected    = false;

   // Control mode constants.
   public static final byte SKIDSTEER    = 0;
   public static final byte SINGLE_STICK = 1;
   public static final int  MIN_SPEED    = 30;
   public static final double CURVE_VAL  = -4;

   /**
    * Set the leftMotorSpeed state variable.
    *
    * @param speed The current left motor speed.
    */
   public void setLeftMotorSpeed(byte speed)
   {
      // The first bit is direction data.
      this.leftMotorSpeed = (byte) (speed & 0x7F);

      // Normalize to an actual negative direction if it the MSB is set.
      if((speed & 0x80) > 0)
      {
         this.leftMotorSpeed *= -1;
      }
   }


   /**
    * Set the rightMotorSpeed state variable.
    *
    * @param speed The current right motor speed.
    */
   public void setRightMotorSpeed(byte speed)
   {
      // The first bit is direction data.
      this.rightMotorSpeed = (byte) (speed & 0x7F);

      // Normalize to an actual negative direction if it the MSB is set.
      if((speed & 0x80) > 0)
      {
         this.rightMotorSpeed *= -1;
      }
   }


   /**
    * Set the servoPosition state variable.
    *
    * @param pos The current servo position, in terms of the PWM value being
    *            fed to it.
    */
   public void setServoPosition(int pos)
   {
      this.servoPosition = pos;
   }


   /**
    * Set the servo motion variable.
    *
    * @param motion Should be one of the constants 
    *               STATIONARY, OUTWARD, or INWARD.
    */
   public void setServoMotion(byte motion)
   {
      this.servoMotion = motion;
   }


   /**
    * Set the actuator motion variable.
    *
    * @param motion Should be one of the constants 
    *               STATIONARY, OUTWARD, or INWARD.
    */
   public void setActuatorMotion(byte motion)
   {
      this.actuatorMotion = motion;
   }


   /**
    * Set the current autostop status.
    *
    * @param autoStop True if autostop is enabled, false otherwise.
    */
   public void setAutoStopIsOn(boolean autoStop)
   {
      this.autoStopIsOn = autoStop;
   }


   /**
    * Set the current autopilot status.
    *
    * @param autoPilot True if autopilot is enabled, false otherwise.
    */
   public void setAutoPilotIsOn(boolean autoPilot)
   {
      this.autoPilotIsOn = autoPilot;
   }


   /**
    * Set the current bridgeMode status.
    *
    * @param bridgeMode True if bride mode is enabled, false otherwise.
    */
   public void setBridgeModeIsOn(boolean bridgeMode)
   {
      this.bridgeModeIsOn = bridgeMode;
   }


   /**
    * Set the current curveVal.
    *
    * @param curveVal The current fscale curve value.
    */
   public void setCurveVal(double curveVal)
   {
      this.curveVal = curveVal;
   }


   /**
    * Set the current horn status.
    *
    * @param horn True if horn is on, false otherwise.
    */
   public void setHornIsOn(boolean horn)
   {
      this.hornIsOn = horn;
   }


   /**
    * Set the current e-magnet status.
    *
    * @param isOn True if the magnet is on, false otherwise.
    */
   public void setEMagnetIsOn(boolean isOn)
   {
      // On a side note, comet isOn was quite a disappointment.
      this.eMagnetIsOn = isOn;
   }


   /**
    * Set the current back left sensor reading.
    *
    * @param reading The current reading of the back left sensor.
    */
   public void setBackLeftSensorData(int reading)
   {
      this.backLeftSensorData = reading;
   }


   /**
    * Set the current front left sensor reading.
    *
    * @param reading The current reading of the front left sensor.
    */
   public void setFrontLeftSensorData(int reading)
   {
      this.frontLeftSensorData = reading;
   }


   /**
    * Set the current back right sensor reading.
    *
    * @param reading The current reading of the back right sensor.
    */
   public void setBackRightSensorData(int reading)
   {
      this.backRightSensorData = reading;
   }


   /**
    * Set the current front right sensor reading.
    *
    * @param reading The current reading of the front right sensor.
    */
   public void setFrontRightSensorData(int reading)
   {
      this.frontRightSensorData = reading;
   }


   /**
    * Set the current front sensor reading.
    *
    * @param reading The current reading of the front sensor.
    */
   public void setFrontSensorData(int reading)
   {
      this.frontSensorData = reading;
   }


   /**
    * Set the current connection status.
    *
    * @param isConnected True if currently connected to the robot,
    *                    false otherwise.
    */
   public void setIsConnected(boolean isConnected)
   {
      this.isConnected = isConnected;
   }


   /**
    * Set the current control mode.
    *
    * @param mode Should be one of the control mode constants.
    */
   public void setControlMode(byte mode)
   {
      this.controlMode = mode;
   }


  /**
    * Get the leftMotorSpeed state variable.
    *
    * @return The current left motor speed.
    */
   public byte getLeftMotorSpeed()
   {
      return this.leftMotorSpeed;
   }


   /**
    * Get the rightMotorSpeed state variable.
    *
    * @return The current right motor speed.
    */
   public byte getRightMotorSpeed()
   {
      return this.rightMotorSpeed;
   }


   /**
    * Get the servoPosition state variable.
    *
    * @return The current servo position, in terms of the PWM value being fed
    *         to it.
    */
   public int getServoPosition()
   {
      return this.servoPosition;
   }


   /**
    * Get the servo motion variable.
    *
    * @return The current servo motion value.
    *         In proper use, should return one of the state variables
    *         STATIONARY, INWARD, or OUTWARD.
    */
   public byte getServoMotion()
   {
      return this.servoMotion;
   }


   /**
    * Get the actuator motion variable.
    *
    * @return The current actuator motion value.
    *         In proper use, should return one of the state variables
    *         STATIONARY, INWARD, or OUTWARD.
    */
   public byte getActuatorMotion()
   {
      return this.actuatorMotion;
   }


   /**
    * Get the current autostop status.
    *
    * @return True if autostop is enabled, false otherwise.
    */
   public boolean getAutoStopIsOn()
   {
      return this.autoStopIsOn;
   }


   /**
    * Get the current autopilot status.
    *
    * @return True if autopilot is enabled, false otherwise.
    */
   public boolean getAutoPilotIsOn()
   {
      return this.autoPilotIsOn;
   }


   /**
    * Get the current bridge mode status.
    *
    * @return True if bridge mode is enabled, false otherwise.
    */
   public boolean getBridgeModeIsOn()
   {
      return this.bridgeModeIsOn;
   }


   /**
    * Get the current curve value for fscale.
    *
    * @return The analog stick curve value.
    */
   public double getCurveVal()
   {
      return this.curveVal;
   }


   /**
    * Get the current horn status.
    *
    * @return True if horn is on, false otherwise.
    */
   public boolean getHornIsOn()
   {
      // On a side note, comet isOn was quite a disappointment.
      return this.hornIsOn;
   }

   
   /**
    * Get the current e-magnet status.
    *
    * @return True if the magnet is on, false otherwise.
    */
   public boolean getEMagnetIsOn()
   {
      return this.eMagnetIsOn;
   }


   /**
    * Get the current back left sensor reading.
    *
    * @return The current reading of the back left sensor.
    */
   public int getBackLeftSensorData()
   {
      return this.backLeftSensorData;
   }


   /**
    * Get the current front left sensor reading.
    *
    * @return The current reading of the front left sensor.
    */
   public int getFrontLeftSensorData()
   {
      return this.frontLeftSensorData;
   }


   /**
    * Get the current back right sensor reading.
    *
    * @return The current reading of the back right sensor.
    */
   public int getBackRightSensorData()
   {
      return this.backRightSensorData;
   }


   /**
    * Get the current front right sensor reading.
    *
    * @return The current reading of the front right sensor.
    */
   public int getFrontRightSensorData()
   {
      return this.frontRightSensorData;
   }


   /**
    * Get the current front sensor reading.
    *
    * @return The current reading of the front sensor.
    */
   public int getFrontSensorData()
   {
      return this.frontSensorData;
   }


   /**
    * Get the current connection status.
    *
    * @return True if currently connected to the robot,
    *         false otherwise.
    */
   public boolean getIsConnected()
   {
      return this.isConnected;
   }


   /**
    * Get the current control mode.
    *
    * @return The current control mode. Should be one of the control
    *         mode constants.
    */
   public byte getControlMode(byte mode)
   {
      return this.controlMode;
   }


   /**
    * Calculates and returns the input checksum.
    *
    * @return The input data checksum.
    */
   public int getInputChecksum()
   {
      return this.frontLeftSensorData ^ this.frontSensorData;
   }


   /**
    * Converts the data stored by this object to JSON for transportation.
    *
    * @return The JSON-encoded data.
    */
   public String toJson()
   {
      StringBuilder s = new StringBuilder();

      // Motors
      s.append("{\"leftMotorSpeed\":");
      s.append(this.leftMotorSpeed);
      s.append(",\"rightMotorSpeed\":");
      s.append(this.rightMotorSpeed);

      // Servo and actuator
      s.append(",\"servoPosition\":");
      s.append(this.servoPosition);
      s.append(",\"servoMotion\":");
      s.append(this.servoMotion);
      s.append(",\"actuatorMotion\":");
      s.append(this.actuatorMotion);

      // Sensors
      s.append(",\"backLeftSensorData\":");
      s.append(this.backLeftSensorData);
      s.append(",\"backRightSensorData\":");
      s.append(this.backRightSensorData);
      s.append(",\"frontLeftSensorData\":");
      s.append(this.frontLeftSensorData);
      s.append(",\"frontRightSensorData\":");
      s.append(this.frontRightSensorData);
      s.append(",\"frontSensorData\":");
      s.append(this.frontSensorData);
      
      // Other miscellany.
      s.append(",\"autoStopIsOn\":");
      s.append(this.autoStopIsOn);
      s.append(",\"autoPilotIsOn\":");
      s.append(this.autoPilotIsOn);
      s.append(",\"hornIsOn\":");
      s.append(this.hornIsOn);
      s.append(",\"eMagnetIsOn\":");
      s.append(this.eMagnetIsOn);
      s.append(",\"isConnected\":");
      s.append(this.isConnected);

      // Control mode
      switch(this.controlMode)
      {
         case SINGLE_STICK:
            s.append(",\"controlMode\":\"SINGLE_STICK\"");
            break;
         case SKIDSTEER:
            s.append(",\"controlMode\":\"SKIDSTEER\"");
            break;
         default:
            s.append(",\"controlMode\":\"ERROR\"");
            System.out.println("ERROR: CONTROL MODE NOT FOUND");
            break;
      }

      // Other local settings.
      s.append(",\"curveVal\":");
      s.append(this.curveVal);
      s.append(",\"bridgeModeIsOn\":");
      s.append(bridgeModeIsOn);

      // Close the string.
      s.append("}");

      // Return the JSON.
      return s.toString();
   }
}
