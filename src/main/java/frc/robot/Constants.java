// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
        public static final class ModuleConstants {
        public static final double kWheelDiameterMeters = 0.1016; //in meters //4 inches
        public static final double kSteerEncoderGearRatio = (150/7); //(150/7): one revolution of wheel equals 21 revolutions on steering motor
        public static final double kDriveEncoderGearRatio = (8.14); //8.14/2*pi: gear ratio*revolution to radians
        public static final double kDriveEncoderRotToMeters = Math.PI * kWheelDiameterMeters/kDriveEncoderGearRatio;
        public static final double kSteerEncoderRot2Rad = (2.0*Math.PI)/kSteerEncoderGearRatio;
        public static final double kDriveEncoderRpm2Mps = kDriveEncoderRotToMeters / 60.0; //rpm = rotations per minute //mps = meters per second
        public static final double kSteerEncoderRPM2RadPerSec = kSteerEncoderRot2Rad / 60.0;
        public static final double kCANCoderCounts = 4096.0; // CANCoders have a resolution of 4096 counts per revolution
        public static final double kAbsoluteEncoderCountsPerMin2Rad = 2.0*Math.PI/kCANCoderCounts; 

        //PID values //TODO: possibly tune PID better
        public static final double kPSteer = 0.6; 
        public static final double kISteer = 0.0; 
        public static final double kDSteer = 0.0; 
        public static final double kFFSteer = 0.0; 

        public static final double kPDrive = 0.002*20.0; 
        public static final double kIDrive = 0.000005*20.0; 
        public static final double kDDrive = 0.0; 
        public static final double kFFDrive = 0.0;

    }
    public static final class DriveConstants {
        //Pigeon ID
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
        

        public static final boolean kFrontLeftSteerEncoderReversed = false;
        public static final boolean kFrontRightSteerEncoderReversed = false;
        public static final boolean kBackLeftSteerEncoderReversed = false;
        public static final boolean kBackRightSteerEncoderReversed = false;

        public static final boolean kFrontLeftAbsoluteEncoderReversed = false;
        public static final boolean kFrontRightAbsoluteEncoderReversed = false;
        public static final boolean kBackLeftAbsoluteEncoderReversed = false;
        public static final boolean kBackRightAbsoluteEncoderReversed = false;
        

        public static final double kFrontLeftAbsoluteEncoderOffsetRad = 5.561;
        public static final double kFrontRightAbsoluteEncoderOffsetRad = 5.502;
        public static final double kBackLeftAbsoluteEncoderOffsetRad = 6.113;
        public static final double kBackRightAbsoluteEncoderOffsetRad = 4.913;
    
        public static final double kWheelBaseMeters = 0.52705; //20.75 inches
        // Distance between right and left wheels
        public static final double kTrackWidthMeters = 0.52705; //20.75 inches
        // Distance between front and back wheels
        public static final double kPhysicalMaxSpeedMps = 3.6576; //meters per second
        public static final double kPhysicalMaxAngularSpeedRps = 3.6576; //radians per second //TODO: Determine actual value


        public static final double kTeleDriveMaxSpeedMps = kPhysicalMaxSpeedMps; // divided by 4 so that we don't drive too fast
        public static final double kTeleopMaxAngularSpeedRps = kPhysicalMaxAngularSpeedRps;
        public static final double kTeleDriveMaxAccelerationUnitsPerSecond = 3;
        public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 3;
        
        
        public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
                new Translation2d(kWheelBaseMeters / 2, -kTrackWidthMeters / 2),
                new Translation2d(kWheelBaseMeters / 2, kTrackWidthMeters / 2),
                new Translation2d(-kWheelBaseMeters / 2, -kTrackWidthMeters / 2),
                new Translation2d(-kWheelBaseMeters / 2, kTrackWidthMeters / 2));
		
		
                      
    }

    public static final class IOConstants {
        public static final int kXboxControllerPort = 0;
    
        public static final double kDeadband = 0.05;
    }

    public static final class AutoConstants {

        public static final double kMaxSpeedMps = 3.0; //TODO: decide actual
        public static final double kMaxAccelerationMetersPerSecondSquared = 1.0; //TODO: decide actual
        public static final double kPThetaController = 0;
        public static final Constraints kThetaControllerConstraints = null;
        public static final double kPYController = 0;
        public static final double kPXController = 0;
        
    }
   
}