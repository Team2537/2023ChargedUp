package frc.robot.constants;

/**
 * This class contains all of the ports for the robot
 */
public final class Ports {
        // ID's of driving motors for each swerve module
        // !! FIX THIS LATER !!
        public static final int FRONT_LEFT_DRIVE = 35;
        public static final int FRONT_RIGHT_DRIVE = 9000;
        public static final int BACK_LEFT_DRIVE = 3;
        public static final int BACK_RIGHT_DRIVE = 4;
    
        // ID's of steering motors for each swerve module
        public static final int FRONT_LEFT_STEER = 13;
        public static final int FRONT_RIGHT_STEER = 6000;
        public static final int BACK_LEFT_STEER = 7;
        public static final int BACK_RIGHT_STEER = 8;
      
        // Encoder port ID for each swerve module
        public static final int FRONT_LEFT_ENCODER = 11;
        public static final int FRONT_RIGHT_ENCODER = 15;
        public static final int BACK_LEFT_ENCODER = 9;
        public static final int BACK_RIGHT_ENCODER = 10;

        // ID's of motors for the arm
        public static final int PIVOT_MOTOR = 12;
        public static final int EXTENSION_MOTOR = 9;

        // Absolute Encoder for pivoting arm
        public static final int ABSOLUTE_ENCODER = 1;
        
        // DIO Ports for sensors
        public static final int TELESCOPE_MAGNET_SENSOR = 0;
        public static final int PIVOT_MAGNET_SENSOR = 2;
}
