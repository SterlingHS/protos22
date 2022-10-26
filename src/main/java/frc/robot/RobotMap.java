package frc.robot;

import java.util.function.DoubleSupplier;

public class RobotMap {
    // Joysitck configuration
    public static int JOYDRIVER_USB_PORT = 0;

    // CAN IDs
    public static int DRIVETRAIN_LEFT_FRONT = 2;
    public static int DRIVETRAIN_LEFT_BACK = 3;
    public static int DRIVETRAIN_RIGHT_FRONT = 4;
    public static int DRIVETRAIN_RIGHT_BACK = 5;

    // Driver configuration
    public static double DRIVER_SLOWDOWN = 1; 
    public static int X_AXIS = 0;
    public static int Y_AXIS = 1;
    public static int Z_AXIS = 4;
    public static double climbingSpeed = .45;
    public static double climbingSpeedBack = -.55;
}

