/**
 * The GamepadConnector connects to an Xbox gamepad and provides access
 * to a gamepad event loop.
 *
 * Nick Overacker
 * Team Atomic
 * Mercury Robotics
 */
 
 import net.java.games.input.*;  // Gamepad
 
 public class GamepadConnector
 {
    /**
     * Constructs a GamepadConnector object, which in turn connects
     * to an Xbox gamepad and creates an event loop, if connected.
     *
     * If no gamepad is connected, the program will exit.
     *
     * @return The connected gamepad.
     */
    public static Controller getGamepad()
    {
       // Disclaimer
       System.out.println("NOTE: It is assumed that an XBox 360 controller "
                        + "has already been connected, and that no other "
                        + "gamepads are connected to the computer. If this "
                        + "is not the case, unexpected behavior may occur.");

       // Access the gamepad library.
       addLibraryPath();
       
       // Set the ControllerEnvironment
       ControllerEnvironment env;
       env = ControllerEnvironment.getDefaultEnvironment();

       // Connect to the XBox controller, and return the event
       // generator returned by the connect method.
       return connect(env);
    }


    /**
     * Adds the path ./lib to the library path, allowing access to JInput.
     */
    private static void addLibraryPath()
    {
       // Change the library path to include the library subdirectory.
       // Hacky, but effective.
       // By the way, this is all an incantation. I can't really say
       // what it's doing, just that it works.
       String p = System.getProperty("java.library.path");
       java.io.File f = new java.io.File("./lib");
       System.setProperty("java.library.path", p + ":" + f.getAbsolutePath());

       try
       {
          java.lang.reflect.Field sysPath;
          sysPath = ClassLoader.class.getDeclaredField("sys_paths");
          sysPath.setAccessible(true);
          sysPath.set(null, null);
          System.out.println("Successfully set sys_paths.");
       }
       catch(Exception e)
       {
          System.out.println("Failed to set sys_paths:");
          e.printStackTrace();
          System.exit(1);
       }
    }


    /**
     * Connects to the XBox 360 controller, and instantiates the
     * gamepad object.
     *
     * @param  env The ControllerEnvironment object to use.
     * @return     The connected gamepad.
     */
    private static Controller connect(ControllerEnvironment env)
    {
       // Get the controller array. There should only be one
       // gamepad connected.
       System.out.println("Trying to connect to gamepad...");
       Controller[] controllers = env.getControllers();

       if(controllers.length != 1)
       {
          System.out.println("Error: " + controllers.length
                           + " controllers found, expected 1.");
          System.exit(1);
       }
       
       // Set the gamepad variable.
       Controller gamepad = controllers[0];
       System.out.println("Selecting " + gamepad);

       // Return the Gamepad.
       return gamepad;
    }
 }
