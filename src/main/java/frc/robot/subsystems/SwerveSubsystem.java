package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.swervedrive.DriveMotor;
import frc.robot.swervedrive.SteerMotor;
import frc.robot.swervedrive.SwerveKinematics;
import frc.robot.swervedrive.SwerveModule;
import frc.robot.swervedrive.SwerveState;

import static frc.robot.constants.Constants.*;
import static frc.robot.constants.Ports.*;

/**
 * The SwerveSubsystem class is a subsystem that controls the swerve drive.
 */
public class SwerveSubsystem extends SubsystemBase {
    

    private final SwerveModule[] m_modules;

    public SwerveSubsystem() {
        SteerMotor steerFL = new SteerMotor(FRONT_LEFT_STEER, FRONT_LEFT_ENCODER, false, FRONT_LEFT_OFFSET);
        SteerMotor steerFR = new SteerMotor(FRONT_RIGHT_STEER, FRONT_RIGHT_ENCODER, false, FRONT_RIGHT_OFFSET);
        SteerMotor steerBL = new SteerMotor(BACK_LEFT_STEER, BACK_LEFT_ENCODER, false, BACK_LEFT_OFFSET);
        SteerMotor steerBR = new SteerMotor(BACK_RIGHT_STEER, BACK_RIGHT_ENCODER, false, BACK_RIGHT_OFFSET);

        DriveMotor driveFL = new DriveMotor(FRONT_LEFT_DRIVE, false);
        DriveMotor driveFR = new DriveMotor(FRONT_RIGHT_DRIVE, false);
        DriveMotor driveBL = new DriveMotor(BACK_LEFT_DRIVE, true);
        DriveMotor driveBR = new DriveMotor(BACK_RIGHT_DRIVE, false);

        SwerveModule swerveFL = new SwerveModule(steerFL, driveFL);
        SwerveModule swerveFR = new SwerveModule(steerFR, driveFR);
        SwerveModule swerveBL = new SwerveModule(steerBL, driveBL);
        SwerveModule swerveBR = new SwerveModule(steerBR, driveBR);

        m_modules = new SwerveModule[] {
            swerveFL,
            swerveFR,
            swerveBL,
            swerveBR
        };
    }
    
    public void setStates(double forward, double strafe, double rotate) {
        forward *= MAX_SPEED * SPEED_MULTIPLIER;
        strafe *= MAX_SPEED * SPEED_MULTIPLIER;
        rotate *= MAX_ANGULAR_VELOCITY * SPEED_MULTIPLIER * 3;

        SwerveState[] desiredStates = SwerveKinematics.getSwerveStates(forward, strafe, rotate);

        for (int i = 0; i < desiredStates.length; i++) {
            m_modules[i].setState(desiredStates[i]);
        }
    }
}
