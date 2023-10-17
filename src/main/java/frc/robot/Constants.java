// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathConstraints;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import swervelib.math.Matter;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import swervelib.parser.PIDFConfig;


/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {



    
    public static final double ROBOT_MASS = (148 - 20.3) * 0.453592; // 32lbs * kg per pound
    public static final Matter CHASSIS    = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
    public static final double LOOP_TIME  = 0.13; //s, 20ms + 110ms sprk max velocity lag

    public static final class Auton
    {

        public static final PIDFConfig xAutoPID     = new PIDFConfig(0.7, 0, 0);
        public static final PIDFConfig yAutoPID     = new PIDFConfig(0.7, 0, 0);
        public static final PIDFConfig angleAutoPID = new PIDFConfig(0.4, 0, 0.01);

        public static final double MAX_SPEED        = 4;
        public static final double MAX_ACCELERATION = 2;
    }

    public static final class Paths{
        public static final PathConstraints constraints = new PathConstraints(1.0, 0.5);

        public static final PathPlannerTrajectory backupTraj = PathPlanner.loadPath("backup", constraints);
        public static final PathPlannerTrajectory blueTwoPieceTraj = PathPlanner.loadPath("Path", constraints);
        public static final PathPlannerTrajectory blueTwoPieceTrajReverse = PathPlanner.loadPath("Reverse Path", constraints);

        public static final PathPlannerTrajectory redTwoPieceTraj = PathPlanner.loadPath("Red Path", constraints);
        public static final PathPlannerTrajectory redTwoPieceTrajReverse = PathPlanner.loadPath("Red Reverse Path", constraints);

        public static final PathPlannerTrajectory testDrive = PathPlanner.loadPath("blueTwoPieceExit", constraints);

    }

    public static final class Drivebase
    {

        // Hold time on motor brakes when disabled
        public static final double WHEEL_LOCK_TIME = 10; // seconds
    }

    public static final class ModuleConstants {
        public static final double kWheelDiameterMeters = 0.1016; // in meters //4 inches
        public static final double kSteerEncoderGearRatio = (150 / 7); // (150/7): one revolution of wheel equals 21
                                                                       // revolutions on steering motor
        public static final double kDriveEncoderGearRatio = (8.14); // 8.14/2*pi: gear ratio*revolution to radians
        public static final double kDriveEncoderRotToMeters = Math.PI * kWheelDiameterMeters / kDriveEncoderGearRatio;
        public static final double kSteerEncoderRot2Rad = (2.0 * Math.PI) / kSteerEncoderGearRatio;
        public static final double kDriveEncoderRpm2Mps = kDriveEncoderRotToMeters / 60.0; // rpm = rotations per minute
                                                                                           // //mps = meters per second
        public static final double kSteerEncoderRPM2RadPerSec = kSteerEncoderRot2Rad / 60.0;

        public static final double kCANCoderCounts = 4096.0; // CANCoders have a resolution of 4096 counts per
                                                             // revolution
        public static final double kAbsoluteEncoderCountsPerMin2Rad = 2.0 * Math.PI / kCANCoderCounts;

        // PID values //TODO: possibly tune PID better
        public static final double kPSteer = 0.6;
        public static final double kISteer = 0.0;
        public static final double kDSteer = 0.0;
        public static final double kFFSteer = 0.0;

        public static final double kPDrive = 0.6;
        public static final double kIDrive = 0.0; //0.000005 * 20.0;
        public static final double kDDrive = 1e-5; // Working at 0

        public static final double kFFDrive = 0.0;

    }


    public static final class IOConstants {
        public static final int kXboxControllerPort = 0;

        public static final double kDeadband = 0.0;
    }

    public static final class AutoConstants {

        public static final double kMaxSpeedMps = 1.0; //DriveConstants.kPhysicalMaxSpeedMps / 3.0
        public static final double kMaxAccelerationMetersPerSecondSquared = 1.0; // DriveConstants.kPhysicalMaxAccelerationMps/ 5

        //TODO: decide whether or not to delete
        public static final double kPThetaController = 0;
        public static final Constraints kThetaControllerConstraints = null;
        public static final double kPYController = 0;
        public static final double kPXController = 0;
        

    }

    public static final class ArmConstants {
        public static final double BOTTOM_ROW_ANGLE = -49.0;
        public static final double BOTTOM_ROW_EXTENSION = 0;

        public static final double MIDDLE_ROW_ANGLE = 6.3;
        public static final double MIDDLE_ROW_EXTENSION = 4.2;

        public static final double TOP_ROW_ANGLE = 26.9;
        public static final double TOP_ROW_EXTENSION = 11;

        public static final double GRAB_ANGLE = -53.2; //from ground
        public static final double GRAB_EXTENSION = 2.45; //from ground

        public static final double SHELF_ANGLE = 17.5;
        public static final double SHELF_EXTENSION = 0.0;

        public static final double HOME_ANGLE = -67;
        public static final double PIVOT_OFFSET = 0;

        public static final double ABSOLUTE_OFFSET = 0.915914;
        public static final double ARM_PIVOT_OFFSET = -72.549;
        public static final double DEFAULT_ANGLE = -72.549;

        // ID's of motors for the arm
        public static final int PIVOT_MOTOR = 12;
        public static final int EXTENSION_MOTOR = 9;

        // Absolute Encoder for pivoting arm
        public static final int ABSOLUTE_ENCODER = 1;

        // DIO Ports for sensors
        public static final int TELESCOPE_MAGNET_SENSOR = 0;
        public static final int PIVOT_MAGNET_SENSOR = 2;
    }

    public static final class LidarConstants {
        public static final double kTargetLow = 0.0;
        public static final double kTargetHigh = 23.0;
        public static final int kLidarReadPort = 5;
        public static final int kLidarTriggerPort = 4;

    }

    public static final class ColorConstants {
        public static final int PURPLE = 3;
        public static final int YELLOW = 2;
        public static final int GREEN = 1;
        public static final int RED = 4;
        public static final int AWESOME = 7;
    }

    public static final class OperatorConstants {
        public static final double LEFT_X_DEADBAND = 0.01;
        public static final double LEFT_Y_DEADBAND = 0.01;
    }

}
