package frc.robot.subsystems;

import com.ctre.phoenix.sensors.Pigeon2;
import com.ctre.phoenix.sensors.Pigeon2Configuration;

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
//creates the logs
    private static DoubleLogEntry headingLog;
    private static DoubleLogEntry drivePosBL;
    private static DoubleLogEntry drivePosFR;
    private static DoubleLogEntry drivePosBR;
    private static DoubleLogEntry drivePosFL;
    private static DoubleLogEntry steerPosBL;
    private static DoubleLogEntry steerPosFR;
    private static DoubleLogEntry steerPosBR;
    private static DoubleLogEntry steerPosFL;
    private static DoubleLogEntry driveVelBL;
    private static DoubleLogEntry driveVelFR;
    private static DoubleLogEntry driveVelBR;
    private static DoubleLogEntry driveVelFL;
    private static DoubleLogEntry steerVelBL;
    private static DoubleLogEntry steerVelFR;
    private static DoubleLogEntry steerVelBR;
    private static DoubleLogEntry steerVelFL;




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
//Adds each log to the data log
        headingLog = new DoubleLogEntry(DataLogManager.getLog(), "/swerveSubsystem/Heading");
        drivePosBL = new DoubleLogEntry(DataLogManager.getLog(), "/swerveSubsystem/Back Left Drive Position");
        drivePosFL = new DoubleLogEntry(DataLogManager.getLog(), "/swerveSubsystem/Front Left Drive Position");
        drivePosBR = new DoubleLogEntry(DataLogManager.getLog(), "/swerveSubsystem/Back Right Drive Position");
        drivePosFR = new DoubleLogEntry(DataLogManager.getLog(), "/swerveSubsystem/Front Right Drive Position");
        steerPosBL = new DoubleLogEntry(DataLogManager.getLog(), "/swerveSubsystem/Back Left Steer Position");
        steerPosFL = new DoubleLogEntry(DataLogManager.getLog(), "/swerveSubsystem/Front Left Steer Position");
        steerPosBR = new DoubleLogEntry(DataLogManager.getLog(), "/swerveSubsystem/Back Right Steer Position");
        steerPosFR = new DoubleLogEntry(DataLogManager.getLog(), "/swerveSubsystem/Front Right Steer Position");
        driveVelBL = new DoubleLogEntry(DataLogManager.getLog(), "/swerveSubsystem/Back Left Drive Velocity");
        driveVelFL = new DoubleLogEntry(DataLogManager.getLog(), "/swerveSubsystem/Front Left Drive Velocity");
        driveVelBR = new DoubleLogEntry(DataLogManager.getLog(), "/swerveSubsystem/Back Right Drive Velocity");
        driveVelFR = new DoubleLogEntry(DataLogManager.getLog(), "/swerveSubsystem/Front Right Drive Velocity");
        steerVelBL = new DoubleLogEntry(DataLogManager.getLog(), "/swerveSubsystem/Back Left Steer Velocity");
        steerVelFL = new DoubleLogEntry(DataLogManager.getLog(), "/swerveSubsystem/Front Left Steer Velocity");
        steerVelBR = new DoubleLogEntry(DataLogManager.getLog(), "/swerveSubsystem/Back Right Steer Velocity");
        steerVelFR = new DoubleLogEntry(DataLogManager.getLog(), "/swerveSubsystem/Front Right Steer Velocity");

        // Setup Shuffleboard
    ShuffleboardTab swerveTab = Shuffleboard.getTab("Swerve Subsystem");
//puts all swerve values that are needed onto the shuffleboard
    swerveTab.addDouble("Heading", () -> getHeading());
    swerveTab.addDouble("Drive Position Front Left", () -> mFrontLeft.getDrivePosition());
    swerveTab.addDouble("Drive Position Back Left", () -> mBackLeft.getDrivePosition());
    swerveTab.addDouble("Drive Position Front Right", () -> mFrontRight.getDrivePosition());
    swerveTab.addDouble("Drive Position Back Right", () -> mBackRight.getDrivePosition());
    swerveTab.addDouble("Steer Position Back Right", () -> mBackRight.getSteerPosition());
    swerveTab.addDouble("Steer Position Front Right", () -> mFrontRight.getSteerPosition());
    swerveTab.addDouble("Steer Position Back Left", () -> mBackLeft.getSteerPosition());
    swerveTab.addDouble("Steer Position Front Left", () -> mFrontLeft.getSteerPosition());
    swerveTab.addDouble("Drive Velocity Front Left", () -> mFrontLeft.getDriveVelocity());
    swerveTab.addDouble("Drive Velocity Back Left", () -> mBackLeft.getDriveVelocity());
    swerveTab.addDouble("Drive Velocity Front Right", () -> mFrontRight.getDriveVelocity());
    swerveTab.addDouble("Drive Velocity Back Right", () -> mBackRight.getDriveVelocity());
    swerveTab.addDouble("Steer Velocity Back Right", () -> mBackRight.getSteerVelocity());
    swerveTab.addDouble("Steer Velocity Front Right", () -> mFrontRight.getSteerVelocity());
    swerveTab.addDouble("Steer Velocity Back Left", () -> mBackLeft.getSteerVelocity());
    swerveTab.addDouble("Steer Velocity Front Left", () -> mFrontLeft.getSteerVelocity());

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

    public void zeroHeading() {
        imu.setYaw(0.0);
    }

    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(-getHeading());
    }

    public Pose2d getPose() {
        return odometer.getPoseMeters();
    }

    //TODO: module positions
    public void resetOdometry(Pose2d pose) {
        odometer.resetPosition(getRotation2d(),getModulePositions(), pose);
    }

    @Override
    public void periodic() {
//Puts every value into the log
    headingLog.append(getHeading());
    drivePosBL.append(mBackLeft.getDrivePosition());
    drivePosFL.append(mFrontLeft.getDrivePosition());
    drivePosBR.append(mBackRight.getDrivePosition());
    drivePosFR.append(mFrontRight.getDrivePosition());
    steerPosBL.append(mBackLeft.getSteerPosition());
    steerPosFL.append(mFrontLeft.getSteerPosition());
    steerPosBR.append(mBackRight.getSteerPosition());
    steerPosFR.append(mFrontRight.getSteerPosition());
    driveVelBL.append(mBackLeft.getDriveVelocity());
    driveVelFL.append(mFrontLeft.getDriveVelocity());
    driveVelBR.append(mBackRight.getDriveVelocity());
    driveVelFR.append(mFrontRight.getDriveVelocity());
    steerVelBL.append(mBackLeft.getSteerVelocity());
    steerVelFL.append(mFrontLeft.getSteerVelocity());
    steerVelBR.append(mBackRight.getSteerVelocity());
    steerVelFR.append(mFrontRight.getSteerVelocity());


        odometer.update(getRotation2d(), getModulePositions());
        SmartDashboard.putNumber("Robot Heading", getHeading());
        SmartDashboard.putString("Robot Location", getPose().getTranslation().toString());
        SmartDashboard.putNumber("Yaw", imu.getYaw());
        SmartDashboard.putNumber("Pitch", imu.getPitch());
        SmartDashboard.putNumber("Roll", imu.getRoll());

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
