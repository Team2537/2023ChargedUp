// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

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

    public static final class DriveConstants {
        // Pigeon ID
        public static final int kPigeonPort = 15;

        // Spark Max IDs
        public static final int kFrontLeftDriveMotorPort = 8;
        public static final int kFrontRightDriveMotorPort = 6;
        public static final int kBackLeftDriveMotorPort = 7;
        public static final int kBackRightDriveMotorPort = 11;

        public static final int kFrontLeftSteerMotorPort = 14;
        public static final int kFrontRightSteerMotorPort = 1;
        public static final int kBackLeftSteerMotorPort = 10;
        public static final int kBackRightSteerMotorPort = 13;

        public static final int kFrontLeftAbsoluteEncoderPort = 5;
        public static final int kFrontRightAbsoluteEncoderPort = 4;
        public static final int kBackLeftAbsoluteEncoderPort = 3;
        public static final int kBackRightAbsoluteEncoderPort = 2;

        public static final boolean kFrontLeftDriveEncoderReversed = false;
        public static final boolean kFrontRightDriveEncoderReversed = true;
        public static final boolean kBackLeftDriveEncoderReversed = false;
        public static final boolean kBackRightDriveEncoderReversed = true;

        public static final boolean kFrontLeftSteerEncoderReversed = true;
        public static final boolean kFrontRightSteerEncoderReversed = true;
        public static final boolean kBackLeftSteerEncoderReversed = true;
        public static final boolean kBackRightSteerEncoderReversed = true;

        public static final boolean kFrontLeftAbsoluteEncoderReversed = false;
        public static final boolean kFrontRightAbsoluteEncoderReversed = false;
        public static final boolean kBackLeftAbsoluteEncoderReversed = false;
        public static final boolean kBackRightAbsoluteEncoderReversed = false;

        public static final double kFrontLeftAbsoluteEncoderOffsetRad = 1.967;
        public static final double kFrontRightAbsoluteEncoderOffsetRad = 0.385;
        public static final double kBackLeftAbsoluteEncoderOffsetRad = 0.687;
        public static final double kBackRightAbsoluteEncoderOffsetRad = 3.335;

        

        public static final double kWheelBaseMeters = 0.52705; // 20.75 inches

        // Distance between right and left wheels
        public static final double kTrackWidthMeters = 0.52705; // 20.75 inches
        // Distance between front and back wheels

        public static final double kPhysicalMaxSpeedMps = 3.0; // meters per second
        public static final double kPhysicalMaxAngularSpeedRps = 10.0; // radians per second

        public static final double kPhysicalMaxAccelerationMps = 5.0; // meters per second squared
        public static final double kPhysicalMaxAngularAccelerationRps = 10.0; // radians per second squared

        public static final double kTeleDriveMaxSpeedMps = kPhysicalMaxSpeedMps; // divided by 4 so that we don't drive
                                                                                 // too fast
        public static final double kTeleAngularMaxSpeedRps = kPhysicalMaxAngularSpeedRps;
        public static final double kTeleDriveMaxAccelerationUnitsPerSecond = kPhysicalMaxAccelerationMps;
        public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = kPhysicalMaxAngularAccelerationRps;

        public static final double kSpeedMultiplier = 0.18;

        public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
                new Translation2d(kWheelBaseMeters / 2, kTrackWidthMeters / 2), // mFrontLeft
                new Translation2d(kWheelBaseMeters / 2, -kTrackWidthMeters / 2), // mFronRight
                new Translation2d(-kWheelBaseMeters / 2, kTrackWidthMeters / 2), // mBackLeft
                new Translation2d(-kWheelBaseMeters / 2, -kTrackWidthMeters / 2)); // mBackRight

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

        public static final double MIDDLE_ROW_ANGLE = 23.2;
        public static final double MIDDLE_ROW_EXTENSION = 1.98;

        public static final double TOP_ROW_ANGLE = 26;
        public static final double TOP_ROW_EXTENSION = 7.3;

        public static final double GRAB_ANGLE = -55.5; //from ground
        public static final double GRAB_EXTENSION = 1.24; //from ground

        public static final double SHELF_ANGLE = 20.5;
        public static final double SHELF_EXTENSION = 2.62;

        public static final double HOME_ANGLE = -55;
        public static final double PIVOT_OFFSET = 0;

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
        public static final double kTargetHigh = 18.0;
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

}
