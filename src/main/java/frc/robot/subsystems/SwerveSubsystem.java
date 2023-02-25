package frc.robot.subsystems;

import com.ctre.phoenix.sensors.Pigeon2;
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
    //declare and instantiate all swerve modules
    private final SwerveModule m_frontLeft = new SwerveModule(
        DriveConstants.kFrontLeftDriveMotorPort,
        DriveConstants.kFrontLeftSteerMotorPort,
        DriveConstants.kFrontLeftDriveEncoderReversed,
        DriveConstants.kFrontLeftSteerEncoderReversed,
        DriveConstants.kFrontLeftAbsoluteEncoderPort,
        DriveConstants.kFrontLeftAbsoluteEncoderOffsetRad,
        DriveConstants.kFrontLeftAbsoluteEncoderReversed,
        "Front Left");

    private final SwerveModule m_frontRight = new SwerveModule(
        DriveConstants.kFrontRightDriveMotorPort,
        DriveConstants.kFrontRightSteerMotorPort,
        DriveConstants.kFrontRightDriveEncoderReversed,
        DriveConstants.kFrontRightSteerEncoderReversed,
        DriveConstants.kFrontRightAbsoluteEncoderPort,
        DriveConstants.kFrontRightAbsoluteEncoderOffsetRad,
        DriveConstants.kFrontRightAbsoluteEncoderReversed,
        "Front Right");

    private final SwerveModule m_backLeft = new SwerveModule(
        DriveConstants.kBackLeftDriveMotorPort,
        DriveConstants.kBackLeftSteerMotorPort,
        DriveConstants.kBackLeftDriveEncoderReversed,
        DriveConstants.kBackLeftSteerEncoderReversed,
        DriveConstants.kBackLeftAbsoluteEncoderPort,
        DriveConstants.kBackLeftAbsoluteEncoderOffsetRad,
        DriveConstants.kBackLeftAbsoluteEncoderReversed,
        "Back Left");

    private final SwerveModule m_backRight = new SwerveModule(
        DriveConstants.kBackRightDriveMotorPort,
        DriveConstants.kBackRightSteerMotorPort,
        DriveConstants.kBackRightDriveEncoderReversed,
        DriveConstants.kBackRightSteerEncoderReversed,
        DriveConstants.kBackRightAbsoluteEncoderPort,
        DriveConstants.kBackRightAbsoluteEncoderOffsetRad,
        DriveConstants.kBackRightAbsoluteEncoderReversed,
        "Back Right");

    //declare and instantiate Inertial Measurement Unit (Pigeon2)
    private Pigeon2 imu = new Pigeon2(DriveConstants.kPigeonPort); 

    private final SwerveDriveOdometry odometer = new SwerveDriveOdometry(DriveConstants.kDriveKinematics, new Rotation2d(0), getModulePositions());
    
    //SwerveSubsystem constructor
    public SwerveSubsystem() {

        new Thread(() -> {
            try {
                Thread.sleep(1000);
                resetImu();
                zeroHeading();
            } catch(Exception ignored) {}        
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
        return Math.IEEEremainder(imu.getYaw(), 360); // Pigeon2 is continuous (yaw value will go past 360 degrees). this converts it to between -180 and 180
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
        odometer.update(getRotation2d(), getModulePositions());
        SmartDashboard.putNumber("Robot Heading", getHeading());
        SmartDashboard.putString("Robot Location", getPose().getTranslation().toString());
        SmartDashboard.putNumber("Yaw", imu.getYaw());
        SmartDashboard.putNumber("Pitch", imu.getPitch());
        SmartDashboard.putNumber("Roll", imu.getRoll());

    }

    public void stopModules() {
        m_frontLeft.stop();
        m_frontRight.stop();
        m_backLeft.stop();
        m_backRight.stop();
    }

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.kPhysicalMaxSpeedMps);
        m_frontLeft.setDesiredState(desiredStates[0]);
        m_frontRight.setDesiredState(desiredStates[1]);
        m_backLeft.setDesiredState(desiredStates[2]);
        m_backRight.setDesiredState(desiredStates[3]);
    }

    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] modulePositions = {
            new SwerveModulePosition(m_frontLeft.getDrivePosition(), m_frontLeft.getState().angle),
            new SwerveModulePosition(m_frontRight.getDrivePosition(), m_frontRight.getState().angle),
            new SwerveModulePosition(m_backLeft.getDrivePosition(), m_backLeft.getState().angle),
            new SwerveModulePosition(m_backRight.getDrivePosition(), m_backRight.getState().angle)
        };

        return modulePositions;
    }

}
