/**
 * Creates and maintains a connection to Team Atomic's Arduino-driven
 * robot, translates gamepad input to robot instructions, and provides
 * an interface for a JavaScript application to monitor the robot state.
 *
 * Nick Overacker
 * Team Atomic
 * Mercury Robotics
 */

import net.java.games.input.Controller;

public class Main
{
   public static void main(String[] args)
   {
      // Make sure an IP and port has been specified.
      if(args.length != 2)
      {
         System.out.println("USAGE: java -cp $CLASSPATH:./lib/jinput-linux.jar"
                       + ":./lib/RXTXcomm.jar:. Main <ROBOT_IP> <ROBOT_PORT>");

         System.out.println("\nIf you are receiving this error, it means that"
               + " you either tried to run the program directly,\nor that the"
               + " arguments are not correctly set in the rn script.");

         System.exit(1);
      }

      // Connect to the gamepad.
      Controller gamepad = GamepadConnector.getGamepad();

      // Create the RobotState.
      RobotState state = new RobotState();
      
      // Create the RobotCommunicator.
      RobotCommunicator comm;
      comm = new RobotCommunicator(gamepad, state,
                                   args[0],                    // host
                                   Integer.parseInt(args[1])); // port
      new Thread(comm).start();

      // Create the StateServer
      StateServer serv = new StateServer(state);
      new Thread(serv).start();

      // That's it! The weakest link is GamepadConnector,
      // because it relies on JInput. JInput is a great
      // library, but it offers no way to recover from
      // (or even detect) a disconnected gamepad.
      //
      // The only cure when problems arise is to restart
      // the program.
   }
}
