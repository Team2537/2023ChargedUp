package frc.robot.subsystems;

import com.ctre.phoenix.sensors.Pigeon2;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class SwerveSubsystem extends SubsystemBase{

    // private static DoubleLogEntry headingLog;
    // private static DoubleLogEntry pitchLog;
    // private static DoubleLogEntry rollLog;
    // private static DoubleLogEntry frontLeftDriveLog;
    // private static DoubleLogEntry frontRightDriveLog;
    // private static DoubleLogEntry backLeftDriveLog;
    // private static DoubleLogEntry backRightDriveLog;
    // private static DoubleLogEntry frontLeftSteerLog;
    // private static DoubleLogEntry frontRightSteerLog;
    // private static DoubleLogEntry backLeftSteerLog;
    // private static DoubleLogEntry backRightSteerLog;
    private static DoubleLogEntry frontLeftVoltageLog;
    private static DoubleLogEntry frontRightVoltageLog;
    private static DoubleLogEntry backLeftVoltageLog;
    private static DoubleLogEntry backRightVoltageLog;


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

        // headingLog = new DoubleLogEntry(DataLogManager.getLog(), "/SwerveSubsystem/Heading");
        // pitchLog = new DoubleLogEntry(DataLogManager.getLog(), "/SwerveSubsystem/Pitch");
        // rollLog = new DoubleLogEntry(DataLogManager.getLog(), "/SwerveSubsystem/Roll");
        // frontLeftDriveLog = new DoubleLogEntry(DataLogManager.getLog(), "/SwerveSubsystem/Front Left Drive Velocity");
        // frontRightDriveLog = new DoubleLogEntry(DataLogManager.getLog(), "/SwerveSubsystem/Front Right Drive Velocity");
        // backLeftDriveLog = new DoubleLogEntry(DataLogManager.getLog(), "/SwerveSubsystem/Back Left Drive Velocity");
        // backRightDriveLog = new DoubleLogEntry(DataLogManager.getLog(), "/SwerveSubsystem/Back Right Drive Velocity");
        // frontLeftSteerLog = new DoubleLogEntry(DataLogManager.getLog(), "/SwerveSubsystem/Front Left Steering Position");
        // frontRightSteerLog = new DoubleLogEntry(DataLogManager.getLog(), "/SwerveSubsystem/Front Right Steering Position");
        // backLeftSteerLog = new DoubleLogEntry(DataLogManager.getLog(), "/SwerveSubsystem/Back Left Steering Position");
        // backRightSteerLog = new DoubleLogEntry(DataLogManager.getLog(), "/SwerveSubsystem/Back Right Steering Position");

        frontLeftVoltageLog = new DoubleLogEntry(DataLogManager.getLog(), "/SwerveSubsystem/Front Left Drive Voltage");
        frontRightVoltageLog = new DoubleLogEntry(DataLogManager.getLog(), "/SwerveSubsystem/Front Right Drive Voltage");
        backLeftVoltageLog = new DoubleLogEntry(DataLogManager.getLog(), "/SwerveSubsystem/Back Left Drive Voltage");
        backRightVoltageLog = new DoubleLogEntry(DataLogManager.getLog(), "/SwerveSubsystem/Back Right Drive Voltage");

        mFrontLeft.resetEncoders();
        mFrontRight.resetEncoders();
        mBackLeft.resetEncoders();
        mBackRight.resetEncoders();

        ShuffleboardTab swerveTab = Shuffleboard.getTab("Swerve State");

        swerveTab.addNumber("Heading", () -> getHeading());
        swerveTab.addNumber("FLV", () -> mFrontLeft.getDriveVelocity());
        swerveTab.addNumber("FRV", () -> mFrontRight.getDriveVelocity());
        swerveTab.addNumber("BLV", () -> mBackLeft.getDriveVelocity());
        swerveTab.addNumber("BRV", () -> mBackRight.getDriveVelocity());
        


        new Thread(() -> {
            try {
                Thread.sleep(1000);
                resetImu();
                zeroHeading();
            } catch(Exception e) {
            }        
        }).start();
    }

    public SwerveModule getFrontLeftModule() {
        return mFrontLeft;
    }

    public SwerveModule getFrontRightModule() {
        return mFrontRight;
    }

    public SwerveModule getBackLeftModule() {
        return mBackLeft;
    }

    public SwerveModule getBackRightModule() {
        return mBackRight;
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

    public Rotation2d getHeadingRotation2d() {
        return Rotation2d.fromDegrees(getHeading());
    }

    public Rotation2d getPitchRotation2d() {
        return Rotation2d.fromDegrees((imu.getPitch()));
    }

    public Pose2d getPose() {
        return odometer.getPoseMeters();
    }

    //TODO: module positions
    public void resetOdometry(Pose2d pose) {
        odometer.resetPosition(getHeadingRotation2d(),getModulePositions(), pose);
    }
/* 
    public void setOdometer(Pose2d pose) {
        odometer.update(getRotation2d(), )
    */

    @Override
    public void periodic() {
        odometer.update(getHeadingRotation2d(), getModulePositions());
         SmartDashboard.putNumber("Robot Heading", getHeading());

         //Puts every value into the log
        //  headingLog.append(getHeading());
        //  pitchLog.append(imu.getPitch());
        //  rollLog.append(imu.getRoll());
        //  frontLeftDriveLog.append(mFrontLeft.getDriveVelocity());
        //  frontRightDriveLog.append(mFrontRight.getDriveVelocity());
        //  backLeftDriveLog.append(mBackLeft.getDriveVelocity());
        //  backRightDriveLog.append(mBackRight.getDriveVelocity());
        //  frontLeftSteerLog.append(mFrontLeft.getSteerPosition());
        //  frontRightSteerLog.append(mFrontRight.getSteerPosition());
        //  backLeftSteerLog.append(mBackLeft.getSteerPosition());
        //  backRightSteerLog.append(mBackRight.getSteerPosition());

        // frontLeftVoltageLog.append(mFrontLeft.getDriveVoltage());
        // frontRightVoltageLog.append(mFrontRight.getDriveVoltage());
        // backLeftVoltageLog.append(mBackLeft.getDriveVoltage());
        // backRightVoltageLog.append(mBackRight.getDriveVoltage());

       
        // // SmartDashboard.putString("Robot Location", getPose().getTranslation().toString());
        SmartDashboard.putNumber("Pitch", imu.getPitch());
        // // SmartDashboard.putNumber("Roll", imu.getRoll());
        SmartDashboard.putNumber("current xPos", getPose().getX());
        SmartDashboard.putNumber("current yPos", getPose().getY());

        //Uncomment if you want to check if absolute and steer encoders are functioning properly
        /* 
        SmartDashboard.putNumber("Front Left Steer Encoder", mFrontLeft.getSteerPosition());
        SmartDashboard.putNumber("Front Right Steer Encoder", mFrontRight.getSteerPosition());
        SmartDashboard.putNumber("Back Left Steer Encoder", mBackLeft.getSteerPosition());
        SmartDashboard.putNumber("Back Right Steer Encoder", mBackRight.getSteerPosition());

        SmartDashboard.putNumber("Front Left Absolute Encoder", mFrontLeft.getAbsoluteEncoder());
        SmartDashboard.putNumber("Front Right Absolute Encoder", mFrontRight.getAbsoluteEncoder());
        SmartDashboard.putNumber("Back Left Absolute Encoder", mBackLeft.getAbsoluteEncoder());
        SmartDashboard.putNumber("Back Right Absolute Encoder", mBackRight.getAbsoluteEncoder());
        */
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