// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    // Frame measurements (meters)
    public static final double WHEEL_BASE = 0.52613;
    public static final double TRACK_WIDTH = 0.52695;

    // ID of driving motors for each swerve module
    public static final int FRONT_LEFT_DRIVE = 1;
    public static final int FRONT_RIGHT_DRIVE = 2;
    public static final int BACK_LEFT_DRIVE = 3;
    public static final int BACK_RIGHT_DRIVE = 4;

    // ID of steering motors for each swerve module
    public static final int FRONT_LEFT_STEER = 5;
    public static final int FRONT_RIGHT_STEER = 6;
    public static final int BACK_LEFT_STEER = 7;
    public static final int BACK_RIGHT_STEER = 8;
  
    // Encoder port ID for each swerve module
    public static final int FRONT_LEFT_ENCODER = 11;
    public static final int FRONT_RIGHT_ENCODER = 12;
    public static final int BACK_LEFT_ENCODER = 9;
    public static final int BACK_RIGHT_ENCODER = 10;
  
    // Absolute encoder offset for each swerve module (in radians)
    public static final double BACK_LEFT_OFFSET = 3.341;
    public static final double BACK_RIGHT_OFFSET = 4.562;
    public static final double FRONT_LEFT_OFFSET = 0.821;
    public static final double FRONT_RIGHT_OFFSET = 0.834;
  
    // Maximum speed of swerve wheel (m/s)
    public static final double MAX_SPEED = 3.6;
}
