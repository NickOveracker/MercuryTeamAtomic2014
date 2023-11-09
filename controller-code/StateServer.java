/**
 * Serves robot state information at localhost:1992 as JSON
 * for the HTML interface to display.
 *
 * Nick Overacker
 * Team Atomic
 * Mercury Robotics
 */

import java.io.*;
import java.net.*;

public class StateServer implements Runnable
{
   private RobotState state;
   private ServerSocket ss;

   /**
    * Hosts a server with dummy data for testing purposes.
    *
    * Dummy data, being intrinsically meaningless, does not
    * warrant many inline comments.
    */
   public static void main(String... args)
   {
      RobotState state = new RobotState();
      StateServer server = new StateServer(state);
      (new Thread(server)).start();

      // Dummy data.
      byte lspeed = 0;
      byte rspeed = -128;
      int servo  = 0;
      int sensor = 0;
      boolean servoUp = true;
      boolean actUp   = true;
      while(true)
      {
         state.setLeftMotorSpeed(lspeed++);
         state.setRightMotorSpeed(rspeed++);
         state.setServoPosition(servoUp ? servo++ : servo--);
         servoUp = (servo == 0) || (servoUp && !(servo == 180));
         state.setServoMotion(servoUp ? RobotState.OUTWARD
                                      : RobotState.INWARD);
         actUp = System.currentTimeMillis()/10000%10<3;
         state.setActuatorMotion(actUp ? RobotState.OUTWARD
                                       : RobotState.INWARD);
         state.setAutoStopIsOn(System.currentTimeMillis()/10000%10<5);
         state.setAutoPilotIsOn(System.currentTimeMillis()/10000%10>8);
         state.setHornIsOn(System.currentTimeMillis()/10000%2==0);
         state.setEMagnetIsOn(System.currentTimeMillis()/10000%4==0);
         state.setBackLeftSensorData(sensor%30);
         state.setFrontLeftSensorData(sensor%50);
         state.setBackRightSensorData(sensor%40);
         state.setFrontRightSensorData(sensor%20);
         state.setFrontSensorData(sensor%200);
         sensor = sensor < 255 ? sensor + 1 : 0;
         state.setControlMode(RobotCommunicator.SINGLE_STICK);
         state.setIsConnected(System.currentTimeMillis()/10000%10<9);

         try
         {
            Thread.sleep(100);
         }
         catch(Exception e)
         {
         }
      }
   }

   /**
    * Constructs a StateServer by initializing the
    * RobotState and ServerSocket.
    *
    * @param _state The RobotState object to poll for the state.
    */
   public StateServer(RobotState _state)
   {
      state = _state;
      try
      {
         ss = new ServerSocket(1992);
      }
      catch(Exception e)
      {
         e.printStackTrace();
         System.out.println("Fatal error.");
         System.exit(1);
      }
   }

   public void run()
   {
      // Declare these while we are waiting on the next
      // request... timing matters in this app.
      PrintWriter out = null;
      Socket socket = null;

      while(true)
      {
         try
         {
            // Wait on the request.
            socket = ss.accept();

            // Do this thing.
            out = new PrintWriter(socket.getOutputStream());
            String json = state.toJson();
            out.print("HTTP/1.1 200 OK\r\n"
                  + "Access-Control-Allow-Origin:*\r\n"
                  + "Content-Type:application/json\r\n"
                  + "Content-Length:" + json.length()  + "\r\n\r\n"
                  + json);
            out.flush();
            socket.close();
         }
         catch(Exception e)
         {
            e.printStackTrace();
         }
      }
   }
}
