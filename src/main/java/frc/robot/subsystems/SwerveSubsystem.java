package frc.robot.subsystems;

import com.ctre.phoenix.sensors.Pigeon2;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

/**
 * The SwerveSubsystem class is the subsystem that controls the Swerve Drivetrain
 */
public class SwerveSubsystem extends SubsystemBase{
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

    /**
     * Constructs the SwerveSubsystem
     */
    public SwerveSubsystem() {

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


    @Override
    public void periodic() {
        odometer.update(getHeadingRotation2d(), getModulePositions());
         SmartDashboard.putNumber("Robot Heading", getHeading());
       
        SmartDashboard.putNumber("Pitch", imu.getPitch());
        SmartDashboard.putNumber("current xPos", getPose().getX());
        SmartDashboard.putNumber("current yPos", getPose().getY());

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