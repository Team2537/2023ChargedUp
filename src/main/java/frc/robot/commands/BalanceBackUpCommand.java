package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.IOConstants;
import frc.robot.subsystems.SwerveSubsystem;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class BalanceBackUpCommand extends CommandBase {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
  private final SwerveSubsystem mSwerveSubsystem;
  private ChassisSpeeds chassisSpeeds = new ChassisSpeeds();
  private double xSpeed;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public BalanceBackUpCommand(SwerveSubsystem swerveSubsystem, double xSpeed) {
      mSwerveSubsystem = swerveSubsystem;
      this.xSpeed = xSpeed;
   

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(mSwerveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // 1. Get real-time joystick inputs


    
    // 4. Construct desired chassis speeds

      // Relative to field
      
      chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
          xSpeed, 0.0, 0.0, mSwerveSubsystem.getHeadingRotation2d());
    // 5. Convert chassis speeds to individual module states
    SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);

    // 6. Output each module states to wheels
    mSwerveSubsystem.setModuleStates(moduleStates);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(Math.abs(mSwerveSubsystem.getPitchRotation2d().getDegrees())>10.0) {
         // 4. Construct desired chassis speeds
      // Relative to field
      chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
          0.0, 0.0, 0.0, mSwerveSubsystem.getHeadingRotation2d());
    // 5. Convert chassis speeds to individual module states
    SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
    // 6. Output each module states to wheels
    mSwerveSubsystem.setModuleStates(moduleStates);
      return true;
    }
    return false;
  }
}
