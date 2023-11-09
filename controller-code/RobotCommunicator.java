/**
 * Creates and maintains a connection to Team Atomic's Arduino-driven
 * robot, translates gamepad input to robot instructions, and provides
 * an interface for a JavaScript application to monitor the robot state.
 *
 * Nick Overacker
 * Team Atomic
 * Mercury Robotics
 */

import net.java.games.input.*;
import java.net.Socket;
import java.io.DataInputStream;
import java.io.PrintStream;
import java.text.DateFormat;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.TimeZone;

public class RobotCommunicator implements Runnable
{
   // Controller and state
   private Controller gamepad;
   private RobotState robotState;

   // Connection variables
   private Socket socket = null;
   private DataInputStream in = null;
   private PrintStream out = null;

   // Control mode constants.
   public static final byte SKIDSTEER    = 0;
   public static final byte SINGLE_STICK = 1;
   public static final int  MIN_SPEED    = 30;

   // Control stuff -- driver side only, not sent to robot
   private byte controlMode   = SINGLE_STICK;
   private double curveVal    = -4;
   private boolean bridgeMode = false;

   // Inputs used
   private Component dPad                 = null; // actuator and servo
   private Component leftStickY           = null; // left motor (skidsteer)
   private Component leftStickX           = null; // for single-stick mode
   private Component rightStickY          = null; // right motor (skidsteer)
   private Component rightShoulderDigital = null; // turn on e-magnet
   private Component leftShoulderDigital  = null; // turn off e-magnet
   private Component rightShoulderAnalog  = null; // horn
   private Component leftShoulderAnalog   = null; // also horn
   private Component aButton              = null; // autostop
   private Component bButton              = null; // bridge mode
   private Component xButton              = null; // lower curve value
   private Component yButton              = null; // raise curve value
   private Component selectButton         = null; // toggle control mode
   private Component startButton          = null; // toggle autopilot

   // An epsilon for normalizing joystick input.
   private static final float EPSILON = 0.08F;

   // Output bytes
   // Note that bytes are signed in Java.
   //
   // The initial byte is always the same (0xAA).
   //
   // The MSB in the motor bytes indicates direction. 0 = forward, 1 = backward.
   // The remaining bits indicate the speed.
   //
   // Controls Byte from MSB to LSB:
   // 7: Signal to power the e-magnet. Not a toggle switch.
   // 6: Turn arm right (away from robot).
   // 5: Turn arm left (toward robot).
   // 4: Lift arm.
   // 3: Lower arm.
   // 2: Autostop
   // 1: Autopilot
   // 0: Horn
   // 
   // Checksum: The XOR of leftMotorByte, rightMotorByte, and controlsByte.
   private final byte INITIAL_BYTE = (byte) 0xAA;
   private byte leftMotorByte      = 0;
   private byte rightMotorByte     = 0;
   private byte controlsByte       = 0;
   private byte checksumByte       = 0;

   // Other IO stuff.
   private static final short INPUT_TIMEOUT   = 800;
   private static final short OUTPUT_INTERVAL = 300;
   private static final short INPUT_SYNC_BYTE = 0xBB;

   // Use these to ensure that we only handle toggle buttons on the rising edge.
   private boolean aButtonLastValue      = false;
   private boolean bButtonLastValue      = false;
   private boolean xButtonLastValue      = false;
   private boolean yButtonLastValue      = false;
   private boolean selectButtonLastValue = false;
   private boolean startButtonLastValue  = false;

   // For logging.
   private DateFormat dfm = new SimpleDateFormat("yyyy-MM-dd HH:mm:ss");

   /**
    * Constructs a RobotCommunicator that polls a given
    * Controller object and updates a given RobotState.
    *
    * @param gamepad The connected gamepad.
    * @param state A RobotState object to allow external access of the
    *              robot's state.
    * @param host The host IP or address to connect to (the Robot's address).
    * @param port The port to connect to the Robot on.
    */
   public RobotCommunicator(Controller gamepad, RobotState state,
                            String host, int port)
   {
      // Hold on to the Controller.
      this.gamepad = gamepad;

      // Hold on to the RobotState.
      this.robotState = state;

      // Set the initial control mode.
      this.robotState.setControlMode(this.controlMode);

      // Create the socket and IO streams.
      try
      {
         this.socket = new Socket(host, port);
         this.in = new DataInputStream(socket.getInputStream());
         this.out = new PrintStream(socket.getOutputStream());
      }
      catch(Exception e)
      {
         System.out.println("Failed to connect to host.");
         System.exit(1);
      }
      
      // For logging.
      dfm.setTimeZone(TimeZone.getTimeZone("America/Chicago"));
   }


   /**
    * Continually polls the gamepad for input and updates the output as
    * necessary.
    */
   public void run()
   {
      getComponents();

      long time;
      long delay;
      long lastSendTime = 0;
      long lastReceiveTime = 0;
      // Continually poll the components.
      while(gamepad.poll())
      {
         // Get the current time...
         time = System.currentTimeMillis();

         // Send output if there it has been updated, or if
         // 300ms have passed since the last output.
         if(updateOutput() || time - lastSendTime > OUTPUT_INTERVAL)
         {
            out.write(INITIAL_BYTE);
            out.write(leftMotorByte);
            out.write(rightMotorByte);
            out.write(controlsByte);
            out.write(checksumByte);
            out.flush();
            lastSendTime = time;
         }

         try
         {
            // See if there's any input data waiting.
            if(in.available() >= 8)
            {
               if(in.readUnsignedByte() == INPUT_SYNC_BYTE)
               {
                  lastReceiveTime = time;
                  robotState.setBackLeftSensorData(in.readUnsignedByte());
                  robotState.setBackRightSensorData(in.readUnsignedByte());
                  robotState.setFrontLeftSensorData(in.readUnsignedByte());
                  robotState.setFrontRightSensorData(in.readUnsignedByte());
                  robotState.setFrontSensorData(in.readUnsignedByte());
                  robotState.setServoPosition(in.readUnsignedByte());
                  robotState.setIsConnected(true);
                  if(robotState.getInputChecksum() != in.readUnsignedByte())
                  {
                     System.out.print(
                        dfm.format(new Date(System.currentTimeMillis()))
                     );
                     System.out.println(": Checksum error.");
                  }
               }
               else 
               {
                  System.out.println("Syncing...");
               }
            }
            else if(time - lastReceiveTime > INPUT_TIMEOUT
                    && robotState.getIsConnected())
            {
               robotState.setIsConnected(false);
               System.out.print(
                  dfm.format(new Date(System.currentTimeMillis()))
               );
               System.out.println(": Input timed out.");
            }

            // We need to sleep for the machine's sake.
            Thread.sleep(20);
         }
         catch(Exception ex)
         {
            robotState.setIsConnected(false);
            System.out.print(
               dfm.format(new Date(System.currentTimeMillis()))
            );
            System.out.println(": Exception.");
         }
      }
   }


   /**
    * Update the output bytes and robot state, and determine whether any
    * changes have occurred.
    *
    * @return True if changes have occurred, false otherwise.
      */
   private boolean updateOutput()
   {
      // Get our input
      float dPadData          = dPad.getPollData();
      float leftStickYData    = leftStickY.getPollData();
      float leftStickXData    = leftStickX.getPollData();
      float rightStickYData   = rightStickY.getPollData();
      float rightShoulderData = rightShoulderDigital.getPollData();
      float leftShoulderData  = leftShoulderDigital.getPollData();
      float hornButtonData    = rightShoulderAnalog.getPollData();
      float aButtonData       = aButton.getPollData();
      float startButtonData   = startButton.getPollData();
      // The following data is handled locally.
      float bButtonData       = bButton.getPollData();
      float xButtonData       = xButton.getPollData();
      float yButtonData       = yButton.getPollData();
      float selectButtonData  = selectButton.getPollData();

      handleServerCommands(bButtonData, xButtonData,
                           yButtonData, selectButtonData);
     
      // Check the other horn button if needed.
      // It is needed if the checked button is unpressed.
      if(hornButtonData == -1F)
      {
         hornButtonData = leftShoulderAnalog.getPollData();
      }

      // Get out temporary output variables. We'll compare these with
      // the real output variables at the end to determine whether
      // any changes have occurred.
      byte tempLeftMotorByte = 0;
      byte tempRightMotorByte = 0;
      byte tempControlsByte = 0;

      switch(controlMode)
      {
         // Get the motor speeds, according to the control mode.
         case SKIDSTEER:
            System.out.println(leftStickYData + ", " + rightStickYData);
            tempLeftMotorByte = getSkidsteerByte(leftStickYData);
            tempRightMotorByte = getSkidsteerByte(rightStickYData);
            break;
         case SINGLE_STICK:
            short motorBytes = 
               getSingleStickBytes(leftStickXData, leftStickYData);
            tempLeftMotorByte = (byte) (motorBytes >> 8);
            tempRightMotorByte = (byte) (motorBytes);
            break;
         default:
            System.out.println("ERROR: CONTROL MODE NOT FOUND");
            System.out.println("SETTING CONTROL MODE TO SKIDSTEER");
            controlMode = SKIDSTEER;
            return updateOutput();
      }

      // Assemble the controls byte.
      tempControlsByte = getControlsByte(leftShoulderData,
                                         rightShoulderData,
                                         dPadData,
                                         aButtonData,
                                         hornButtonData,
                                         startButtonData);

      // Compare current and original bytes.
      if(tempLeftMotorByte != leftMotorByte ||
         tempRightMotorByte != rightMotorByte ||
         tempControlsByte != controlsByte)
      {
         // Update if there is a change.
         leftMotorByte = tempLeftMotorByte;
         rightMotorByte = tempRightMotorByte;
         controlsByte = tempControlsByte;
         checksumByte = (byte) (leftMotorByte ^ rightMotorByte ^ controlsByte);

         // Update the state object
         updateRobotState();

         return true; // yes, there was a change.
      }
      return false; // no, there was not a change.
   }


   /**
    * Returns the left and right motor bytes in single-stick
    * control mode. The most significant byte is the left
    * motor byte, and the least significant byte is the right
    * motor byte. This is mostly pirated from Matthew Spinks's
    * code.
    *
    * @param xInput The horizontal axis data for the analog stick.
    * @param yInput The vertical axis data for the analog stick.
    * @return The left and right motor bytes concatenated into
    *           one short. The left motor byte is the most
    *           significant byte.
    */
   private short getSingleStickBytes(float xInput, float yInput)
   {
      if(yInput < EPSILON && yInput > -EPSILON && xInput < EPSILON && xInput > -EPSILON)
      {
         // The stick is probably not even being moved.
         return (short) 0;
      }

      double lMotorFloat = 0;
      double rMotorFloat = 0;
      
      yInput = -yInput; // Inverted. Because Matt did that.

      // Trig functions use RADIANS by default.
      double angle = Math.atan2(yInput, xInput);
      double magnitude = Math.sqrt(xInput*xInput + yInput*yInput);
      double stickScale = Math.cos(2*angle);

      // remap the magnitude eliminating any slack in the joystick
      // think about where the function should start mapping joystick values.
      // It begins at the point (tolerance, tolerance). Now find the magnitude
      // of that.
      // mag = sqrt((tolerance^2) + (tolerance^2))
      // which simplifies to:
      // mag = sqrt(2*(tolerance^2))
      // That's our starting point. i.i. our new zero point.
      double inMin = Math.sqrt(2*EPSILON*EPSILON);
      double inMax = 1;
      double outMin = 0;
      double outMax = 1;
      magnitude =
         (
            (magnitude - inMin) *
            (outMax - outMin) /
            (inMax - inMin) +
            outMin
         );

      // joystick is not a perfect circle, therefore ignore anything
      // greater than 1.0 or less than zero
      if(magnitude > 1.0)
         magnitude = 1.0;
      if(magnitude < 0)
         magnitude = 0;

      if(angle > 0 && angle < Math.PI/2) // Quadrant I
      {
         lMotorFloat = magnitude;
         rMotorFloat = -1 * stickScale * magnitude;
      }
      else if(angle > Math.PI/2 && angle < Math.PI) // Quadrant II
      {
         rMotorFloat = magnitude;
         lMotorFloat = -1 * stickScale * magnitude;
      }
      else if(angle > -Math.PI && angle < -Math.PI/2) // Quadrant III
      {
         lMotorFloat = -1 * magnitude;
         rMotorFloat = stickScale * magnitude;
      }
      else if(angle > -Math.PI/2 && angle < 0) // Quadrant IV
      {
         rMotorFloat = -1 * magnitude;
         lMotorFloat = stickScale * magnitude;
      }

      // Get the sign bits.
      boolean leftForward = lMotorFloat > 0;
      boolean rightForward = rMotorFloat > 0;

      // Get the absolute values of the motor floats.
      lMotorFloat = lMotorFloat > 0 ? lMotorFloat : -lMotorFloat;
      rMotorFloat = rMotorFloat > 0 ? rMotorFloat : -rMotorFloat;
      
      // Get the left byte.
      short retVal = 
         (short) (callFscale(lMotorFloat));
      retVal <<= 8;
      retVal |= 
         (byte) (callFscale(rMotorFloat));

      if(leftForward)
      {
         retVal &= 0x7FFF;
      }
      else
      {
         retVal |= 0x8000;
      }
      if(rightForward)
      {
         retVal &= 0xFF7F;
      }
      else
      {
         retVal |= 0x0080;
      }

      return retVal;
   }


   /**
    * Returns the appropriate motor bytes for skidsteer controls.
    *
    * @param input The analog input for the motor.
    * @return The appropriate output byte for the motor.
    */
   private byte getSkidsteerByte(float input)
   {
      if(input < EPSILON && input > 0-EPSILON)
      {
         // The stick is probably not even being moved.
         return (byte) 0;
      }
      else
      {
         // Get the speed for the last seven bits.
         float absInput = input > 0 ? input : input*-1;
         byte speed = 
            (byte) (callFscale(absInput));

         // Now get the direction as the 8th bit.
         // Up on the analog stick is negative, and we want
         // up to be "forward" (0 in MSB).
         byte dir = input > 0 ? (byte) (128): (byte) 0; 

         // Now put it all together.
         return (byte) (dir | speed);
      }
   }


   /**
    * Returns the appropriate controls byte given a D-Pad input.
    *
    * @param leftBumpInput The left shoulder digital input.   // e-magnet OFF
    * @param rightBumpInput The right-shoulder digital input. // e-magnet ON
    * @param dPadInput The D-Pad input.                       // actuator&servo
    * @param hornInput The button for the horn (right shoulder analog).
    * @return The appropriate controls byte.
    */
   private byte getControlsByte(float leftBumpInput,
                                float rightBumpInput,
                                float dPadInput,
                                float aButtonInput,
                                float hornInput,
                                float startButtonInput)
   {
      byte tempControlsByte = 0;

      // E-magnet being on has priority.
      // If neither ON nor OFF is pressed, just hold the old value.
      if(rightBumpInput != 0f) // ON button
      {
         tempControlsByte = (byte) 128;
      }
      else if(leftBumpInput != 0f) // OFF button
      {
         tempControlsByte = (byte) 0;
      }
      else
      {
         tempControlsByte = robotState.getEMagnetIsOn() ? (byte) 128 : (byte) 0;
      }

      // D-Pad left/right: turn servo left/right.
      // D-Pad up/down: move actuator up/down.
      // Ignore the case where the D-Pad is not pressed - this is all zeros.

      /** NOTE: THE SERVO TURNS IN THE OPPOSITE DIRECTION OF THE SPECIFICATIONS AS OF COMPETITION DAY! **/
      if(dPadInput == Component.POV.LEFT) {
         tempControlsByte = (byte) (tempControlsByte | 64);
      }
      else if(dPadInput == Component.POV.RIGHT) {
         tempControlsByte = (byte) (tempControlsByte | 32);
      }
      else if(dPadInput == Component.POV.UP) {
         tempControlsByte = (byte) (tempControlsByte | 16);
      }
      else if(dPadInput == Component.POV.DOWN) {
         tempControlsByte = (byte) (tempControlsByte | 8);
      }
      /*else if(dPadInput == Component.POV.UP_RIGHT) {
         tempControlsByte = (byte) (tempControlsByte | 64 | 16);
      }
      else if(dPadInput == Component.POV.UP_LEFT) {
         tempControlsByte = (byte) (tempControlsByte | 64 | 8);
      }
      else if(dPadInput == Component.POV.DOWN_RIGHT) {
         tempControlsByte = (byte) (tempControlsByte | 64 | 8);
      }
      else if(dPadInput == Component.POV.DOWN_LEFT) {
         tempControlsByte = (byte) (tempControlsByte | 32 | 8);
      }*/

      // Autostop (rising edge only)
      if(aButtonInput != 0F && !aButtonLastValue)
      {
         // Toggle.
         tempControlsByte |= robotState.getAutoStopIsOn() ? 0 : 4;
      }
      else
      {
         tempControlsByte |= robotState.getAutoStopIsOn() ? 4 : 0;
      }

      // Autopilot (rising edge only)
      if(startButtonInput != 0F && !startButtonLastValue)
      {
         // Toggle.
         //tempControlsByte |= robotState.getAutoPilotIsOn() ? 0 : 2;
      }
      else
      {
         tempControlsByte |= robotState.getAutoPilotIsOn() ? 2 : 0;
      }

      // Meep meep!
      if(hornInput != -1F)
      {
         tempControlsByte = (byte) (tempControlsByte | 1);
      }

      // Set the "lastValue" variables for A and Start.
      aButtonLastValue = aButtonInput != 0F;
      startButtonLastValue = startButtonInput != 0F;

      // We're done. The last three bits are reserved for now.
      return tempControlsByte;
   }


   /**
    * A remapping function with logarithmic scaling.
    *
    * Based on a function written by Gred Shakar, and subsequently
    * modified by Paul Badger (2007) and Matthew Spinks (2012).
    *
    * Comments in the method are carried over directly from Matthew's
    * version.
    *
    * Adapted in 2014 by Nick Overacker.
    *
    * @param inputValue  he value to remap.
    * @param originalMin The original minimum value.
    * @param originalMax The original maximum value.
    * @param newBegin    The new minimum value.
    * @param newEnd      The new maximum value.
    * @param curveValue  The "sharpness" of the remapping curve.
    * @return A curved remapping of the input value.
    */
   double fscale(double inputValue, double originalMin, double originalMax,
         double newBegin, double newEnd, double curveValue)
   {
      double originalRange = 0;
      double newRange = 0;
      double zeroRefCurVal = 0;
      double normalizedCurVal = 0;
      double rangedValue = 0;
      boolean invFlag = false;

      // Condition curve parameter
      // Limit range
      if(curveValue > 10)
      {
         curveValue = 10;
      }
      else if(curveValue < -10)
      {
         curveValue = -10;
      }

      // Invert and scale - this seems more intuitive.
      // Positive numbers give more weight to high end on output.
      curveValue *= -.1;
      // Convert linear scale into logarithmic exponent for other pow function.
      curveValue = Math.pow(10, curveValue);

      // Zero reference the values.
      originalRange = originalMax - originalMin;

      if(newEnd > newBegin)
      {
         newRange = newEnd - newBegin;
      }
      else
      {
         newRange = newBegin - newEnd;
         invFlag = true;
      }
      
      zeroRefCurVal = inputValue - originalMin;
      // Normalize to 0 - 1 double.
      normalizedCurVal = zeroRefCurVal / originalRange; 

      if(invFlag == false)
      {
         rangedValue = Math.pow(normalizedCurVal, curveValue) * newRange
            + newBegin;
      }
      else // invert the ranges
      {
         rangedValue = newBegin - Math.pow(normalizedCurVal, curveValue)
            * newRange;
      }

      return rangedValue;
   }


   /**
    * Calls fscale with appropriate parameters.
    * 
    * I don't want to modify fscale, since it is a good general-purpose
    * function. So instead, I am using this to set the parameters.
    *
    * @param input The input value to scale.
    * @return The scaled value.
    */
   double callFscale(double input)
   {
      // Note the dependence on bridgeMode.
      return fscale(input, EPSILON, 1, MIN_SPEED, bridgeMode ? 50 : 127, curveVal);
   }


   /**
    * Updates the RobotState object to reflect the current command being sent.
    */
   private void updateRobotState()
   {
      // Motors
      robotState.setLeftMotorSpeed(leftMotorByte);
      robotState.setRightMotorSpeed(rightMotorByte);

      // Servo
      if((controlsByte & 64) == (controlsByte & 32) && (controlsByte & 64) == 0)
      {
         robotState.setServoMotion(RobotState.STATIONARY);
      }
      else if((controlsByte & 64) == 64)
      {
         robotState.setServoMotion(RobotState.OUTWARD);
      }
      else
      {
         robotState.setServoMotion(RobotState.INWARD);
      }

      // Actuator
      if((controlsByte & 16) == (controlsByte & 8) && (controlsByte & 16) == 0)
      {
         robotState.setActuatorMotion(RobotState.STATIONARY);
      }
      else if((controlsByte & 16) == 16)
      {
         robotState.setActuatorMotion(RobotState.OUTWARD);
      }
      else
      {
         robotState.setActuatorMotion(RobotState.INWARD);
      }

      // Booleans
      robotState.setEMagnetIsOn((controlsByte&128) != 0);
      robotState.setAutoStopIsOn((controlsByte&4) != 0);
      robotState.setAutoPilotIsOn((controlsByte&2) != 0);
      robotState.setHornIsOn((controlsByte&1) != 0);

      return;
   }

   
   /**
    * Handle server-side commands, such as modifying the curve value
    * and setting bridge mode.
    */
   private void handleServerCommands(float bButtonData, float xButtonData,
                                     float yButtonData, float selectButtonData)
   {
      // Set the control mode on the rising edge of the select button.
      if(selectButtonData != 0F && !selectButtonLastValue)
      {
         controlMode = controlMode == SINGLE_STICK ? SKIDSTEER: SINGLE_STICK;
      }

      // Toggle bridge mode on the rising edge of the B button.
      if(bButtonData != 0F && !bButtonLastValue)
      {
         bridgeMode = !bridgeMode;
      }

      // Decrease curve value on the rising edge of the X button.
      if(xButtonData != 0F && !xButtonLastValue)
      {
         curveVal -= 0.5;
      }

      // Increase curve value on the rising edge of the Y button.
      if(yButtonData != 0F && !yButtonLastValue)
      {
         curveVal += 0.5;
      }

      selectButtonLastValue = selectButtonData != 0F;
      bButtonLastValue = bButtonData != 0F;
      xButtonLastValue = xButtonData != 0F;
      yButtonLastValue = yButtonData != 0F;

      // Set the state.
      // This cannot be done in setRobotState, because that is
      // not always updated instantaneously.
      robotState.setCurveVal(curveVal);
      robotState.setBridgeModeIsOn(bridgeMode);
      robotState.setControlMode(controlMode);

   }

   
   /**
    * Initializes all of the Component objects.
    */
   private void getComponents()
   {
      // Available components for Xbox controller, as named by getComponents().
      // "A", "B", "X", "Y"                                          (Digital)
      // "Left Thumb", "Right Thumb" (digital shoulder buttons)      (Digital)
      // "Select", "Unknown" (Unknown is "start".)                   (Digital)
      // "Mode" (The connect button.)                                (Digital)
      // "Left Thumb 3", "Right Thumb 3" (joystick clickers.)        (Digital)
      // "x", "y" (left analog stick X and Y axes.)                   (Analog)
      // "rx", "ry" (right analog stick X and Y axes.)                (Analog)
      // "z", "rz" (left and right analog shoulder buttons.)          (Analog)
      // "pov" (D-pad.)                                        (Component.POV)

      // Get the components
      Component[] components = gamepad.getComponents();

      // Assign the components.
      for(int i = 0; i < components.length; i++)
      {
         String s = components[i].toString();
         if(s.equals("Right Thumb"))
         {
            rightShoulderDigital = components[i];
         }
         else if(s.equals("Left Thumb"))
         {
            leftShoulderDigital = components[i];
         }
         else if(s.equals("y"))
         {
            leftStickY = components[i];
         }
         else if(s.equals("x"))
         {
            leftStickX = components[i];
         }
         else if(s.equals("ry"))
         {
            rightStickY = components[i];
         }
         else if(s.equals("pov"))
         {
            dPad = components[i];
         }
         else if(s.equals("rz"))
         {
            rightShoulderAnalog = components[i];
         }
         else if(s.equals("z"))
         {
            leftShoulderAnalog = components[i];
         }
         else if(s.equals("A"))
         {
            aButton = components[i];
         }
         else if(s.equals("B"))
         {
            bButton = components[i];
         }
         else if(s.equals("X"))
         {
            xButton = components[i];
         }
         else if(s.equals("Y"))
         {
            yButton = components[i];
         }
         else if(s.equals("Select"))
         {
            selectButton = components[i];
         }
         else if(s.equals("Unknown"))
         {
            startButton = components[i];
         }
      }
   }
}
