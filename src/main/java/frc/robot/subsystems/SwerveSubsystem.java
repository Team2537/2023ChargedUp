package frc.robot.subsystems;

import com.ctre.phoenix.sensors.Pigeon2;
import com.ctre.phoenix.sensors.Pigeon2Configuration;

import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class SwerveSubsystem extends SubsystemBase{

// sets the logs up, naming them and creating the log entries
    private final DoubleLogEntry HeadingLog;
    private final DoubleLogEntry PitchLog;
    private final DoubleLogEntry RollLog;
    private final DoubleLogEntry FLDriveLog;
    private final DoubleLogEntry FRDriveLog;
    private final DoubleLogEntry BLDriveLog;
    private final DoubleLogEntry BRDriveLog;
    private final DoubleLogEntry FLSteerLog;
    private final DoubleLogEntry FRSteerLog;
    private final DoubleLogEntry BLSteerLog;
    private final DoubleLogEntry BRSteerLog;




    //declare and instantiate all swerve modules
    private final SwerveModule mFrontLeft = new SwerveModule(
        DriveConstants.kFrontLeftDriveMotorPort,
        DriveConstants.kFrontLeftSteerMotorPort,
        DriveConstants.kFrontLeftDriveEncoderReversed,
        DriveConstants.kFrontLeftSteerEncoderReversed,
        DriveConstants.kFrontLeftAbsoluteEncoderPort,
        DriveConstants.kFrontLeftAbsoluteEncoderOffsetRad,
        DriveConstants.kFrontLeftAbsoluteEncoderReversed);

    private final SwerveModule mFrontRight = new SwerveModule(
        DriveConstants.kFrontRightDriveMotorPort,
        DriveConstants.kFrontRightSteerMotorPort,
        DriveConstants.kFrontRightDriveEncoderReversed,
        DriveConstants.kFrontRightSteerEncoderReversed,
        DriveConstants.kFrontRightAbsoluteEncoderPort,
        DriveConstants.kFrontRightAbsoluteEncoderOffsetRad,
        DriveConstants.kFrontRightAbsoluteEncoderReversed);

    private final SwerveModule mBackLeft = new SwerveModule(
        DriveConstants.kBackLeftDriveMotorPort,
        DriveConstants.kBackLeftSteerMotorPort,
        DriveConstants.kBackLeftDriveEncoderReversed,
        DriveConstants.kBackLeftSteerEncoderReversed,
        DriveConstants.kBackLeftAbsoluteEncoderPort,
        DriveConstants.kBackLeftAbsoluteEncoderOffsetRad,
        DriveConstants.kBackLeftAbsoluteEncoderReversed);

    private final SwerveModule mBackRight = new SwerveModule(
        DriveConstants.kBackRightDriveMotorPort,
        DriveConstants.kBackRightSteerMotorPort,
        DriveConstants.kBackRightDriveEncoderReversed,
        DriveConstants.kBackRightSteerEncoderReversed,
        DriveConstants.kBackRightAbsoluteEncoderPort,
        DriveConstants.kBackRightAbsoluteEncoderOffsetRad,
        DriveConstants.kBackRightAbsoluteEncoderReversed);

    //declare and instantiate Inertial Measurement Unit (Pigeon2)
    private Pigeon2 imu = new Pigeon2(DriveConstants.kPigeonPort); 

    private final SwerveDriveOdometry odometer = new SwerveDriveOdometry(DriveConstants.kDriveKinematics, new Rotation2d(0), getModulePositions());
    //SwerveSubsystem constructor
    public SwerveSubsystem() {

// adds the log values to the log
        HeadingLog = new DoubleLogEntry(DataLogManager.getLog(), "/SwerveSubsystem/Heading");
        PitchLog = new DoubleLogEntry(DataLogManager.getLog(), "/SwerveSubsystem/Pitch");
        RollLog = new DoubleLogEntry(DataLogManager.getLog(), "/SwerveSubsystem/Roll");
        FLDriveLog = new DoubleLogEntry(DataLogManager.getLog(), "/SwerveSubsystem/Front Left Drive Velocity");
        FRDriveLog = new DoubleLogEntry(DataLogManager.getLog(), "/SwerveSubsystem/Front Right Drive Velocity");
        BLDriveLog = new DoubleLogEntry(DataLogManager.getLog(), "/SwerveSubsystem/Back Left Drive Velocity");
        BRDriveLog = new DoubleLogEntry(DataLogManager.getLog(), "/SwerveSubsystem/Back Right Drive Velocity");
        FLSteerLog = new DoubleLogEntry(DataLogManager.getLog(), "/SwerveSubsystem/Front Left Steering Position");
        FRSteerLog = new DoubleLogEntry(DataLogManager.getLog(), "/SwerveSubsystem/Front Right Steering Position");
        BLSteerLog = new DoubleLogEntry(DataLogManager.getLog(), "/SwerveSubsystem/Back Left Steering Position");
        BRSteerLog = new DoubleLogEntry(DataLogManager.getLog(), "/SwerveSubsystem/Back Right Steering Position");

        new Thread(() -> {
            try {
                Thread.sleep(1000);
                resetImu();
                zeroHeading();
            } catch(Exception e) {
            }        
        }).start();
    }




    public void resetImu() {
        /* 
        Pigeon2Configuration config = new Pigeon2Configuration();
        config.MountPoseYaw = 0;
        config.MountPosePitch = 0;
        config.MountPoseRoll = 0;
        imu.configAllSettings(config);
        */

        imu.configMountPose(0.0, 0.0, 0.0);
    }

    public double getHeading() {
        return Math.IEEEremainder(imu.getYaw(), 360); //Pigeon2 is continuous (yaw value will go past 360 degrees). this converts it to between -180 and 180
    }

    public void setHeading(double angleDeg) {
        imu.setYaw(angleDeg);
    }

    public void zeroHeading() {
        setHeading(0.0);
    }

    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(getHeading());
    }

    public Pose2d getPose() {
        return odometer.getPoseMeters();
    }

    //TODO: module positions
    public void resetOdometry(Pose2d pose) {
        odometer.resetPosition(getRotation2d(),getModulePositions(), pose);
    }
/* 
    public void setOdometer(Pose2d pose) {
        odometer.update(getRotation2d(), )
    */

    @Override
    public void periodic() {

// appends the values to the logs, I couldnt find the steer values, so they dont work for now... Please comment this subsystem.
        HeadingLog.append(getHeading());
        PitchLog.append(imu.getPitch());
        RollLog.append(imu.getRoll());
        FLDriveLog.append(mFrontLeft.getDriveVelocity());
        FRDriveLog.append(mFrontRight.getDriveVelocity());
        BLDriveLog.append(mBackLeft.getDriveVelocity());
        BRDriveLog.append(mBackRight.getDriveVelocity());
        FLSteerLog.append(mFrontLeft.getSteerPosition());
        FRSteerLog.append(mFrontRight.getSteerPosition());
        BLSteerLog.append(mBackLeft.getSteerPosition());
        BRSteerLog.append(mBackRight.getSteerPosition());

        odometer.update(getRotation2d(), getModulePositions());
         SmartDashboard.putNumber("Robot Heading", getHeading());

       
        // SmartDashboard.putString("Robot Location", getPose().getTranslation().toString());
        // SmartDashboard.putNumber("Pitch", imu.getPitch());
        // SmartDashboard.putNumber("Roll", imu.getRoll());
        SmartDashboard.putNumber("xPos", getPose().getX());
        SmartDashboard.putNumber("yPos", getPose().getY());
    }



    public void stopModules() {
        mFrontLeft.stop();
        mFrontRight.stop();
        mBackLeft.stop();
        mBackRight.stop();
    }

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.kPhysicalMaxSpeedMps);
        mFrontLeft.setDesiredState(desiredStates[0]);
        mFrontRight.setDesiredState(desiredStates[1]);
        mBackLeft.setDesiredState(desiredStates[2]);
        mBackRight.setDesiredState(desiredStates[3]);
    }

    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] modulePositions = {
            new SwerveModulePosition(mFrontLeft.getDrivePosition(), mFrontLeft.getState().angle),
            new SwerveModulePosition(mFrontRight.getDrivePosition(), mFrontRight.getState().angle),
            new SwerveModulePosition(mBackLeft.getDrivePosition(), mBackLeft.getState().angle),
            new SwerveModulePosition(mBackRight.getDrivePosition(), mBackRight.getState().angle)
        };

        return modulePositions;
    }

}
