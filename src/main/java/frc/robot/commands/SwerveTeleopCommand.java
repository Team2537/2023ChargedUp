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

public class SwerveTeleopCommand extends CommandBase {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
  private final SwerveSubsystem mSwerveSubsystem;
  private final Supplier<Double> xSpdFunction, ySpdFunction, turningSpdFunction;
  private final Supplier<Boolean> fieldOrientedFunction, fastSpeedFunction;
  private final SlewRateLimiter xLimiter, yLimiter, turningLimiter;
  private final double joystickAccel = 10 * 0.02;
  private double activeAccel = joystickAccel;
  private ChassisSpeeds chassisSpeeds = new ChassisSpeeds();

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public SwerveTeleopCommand(SwerveSubsystem swerveSubsystem, Supplier<Double> xSpdFunction,
      Supplier<Double> ySpdFunction, Supplier<Double> turningSpdFunction,
      Supplier<Boolean> fieldOrientedFunction, Supplier<Boolean> fastSpeedFunction) {
    mSwerveSubsystem = swerveSubsystem;
    this.xSpdFunction = xSpdFunction;
    this.ySpdFunction = ySpdFunction;
    this.turningSpdFunction = turningSpdFunction;
    this.fieldOrientedFunction = fieldOrientedFunction;
    this.fastSpeedFunction = fastSpeedFunction;
    xLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
    yLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
    turningLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAngularAccelerationUnitsPerSecond);
    ShuffleboardTab tab = Shuffleboard.getTab("Swerve State");
    tab.addBoolean("Slow Mode", () -> this.fastSpeedFunction.get());

    tab.addNumber("X Speed", () -> xSpeed);
    tab.addNumber("Y Speed", () -> ySpeed);

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(mSwerveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  double xSpeed, ySpeed;

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // 1. Get real-time joystick inputs

    xSpeed = Math.pow(xSpdFunction.get(), 3);

    // xSpeed+= .2*Math.signum(xSpeed);
    ySpeed = Math.pow(ySpdFunction.get(), 3);
    // ySpeed+= .2*Math.signum(ySpeed);
    double turningSpeed = turningSpdFunction.get();

    // activeAccel = Math.min(joystickAccel / Math.pow(Math.sqrt(Math.pow(xSpeed, 2) + Math.pow(ySpeed, 2)), 0.4), 0.35);

    // 2. Apply deadband
    xSpeed = Math.abs(xSpeed) > IOConstants.kDeadband ? xSpeed : 0.0;
    ySpeed = Math.abs(ySpeed) > IOConstants.kDeadband ? ySpeed : 0.0;
    turningSpeed = Math.abs(turningSpeed) > IOConstants.kDeadband ? turningSpeed : 0.0;

    // 3. Make the driving smoother
    // xSpeed = xLimiter.calculate(xSpeed * DriveConstants.kTeleDriveMaxSpeedMps);
    // ySpeed = yLimiter.calculate(ySpeed * DriveConstants.kTeleDriveMaxSpeedMps);
    // turningSpeed = turningLimiter.calculate(turningSpeed * DriveConstants.kTeleAngularMaxSpeedRps);

    // SmartDashboard.putNumber("Turning speed", turningSpeed);

    if (!fastSpeedFunction.get()) {
      xSpeed *= DriveConstants.kSpeedMultiplier;
      ySpeed *= DriveConstants.kSpeedMultiplier;
      turningSpeed *= DriveConstants.kSpeedMultiplier;
    }
    else {
      xSpeed *= 1.5;
      ySpeed *= 1.5;
      turningSpeed *= 1.5;
    }

    // if (Math.abs(xSpeed - chassisSpeeds.vxMetersPerSecond) > activeAccel){
    //   xSpeed = chassisSpeeds.vxMetersPerSecond + (activeAccel * Math.signum(xSpeed - chassisSpeeds.vxMetersPerSecond));
    // }
    // if (Math.abs(ySpeed - chassisSpeeds.vyMetersPerSecond) > activeAccel){
    //   ySpeed = chassisSpeeds.vyMetersPerSecond + (activeAccel * Math.signum(ySpeed - chassisSpeeds.vyMetersPerSecond));
    // }
      SmartDashboard.putNumber("XSpeed", xSpeed);
      SmartDashboard.putNumber("YSpeed", ySpeed);
    
    // 4. Construct desired chassis speeds
    if (fieldOrientedFunction.get()) {
      // Relative to field
      //
      chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
          xSpeed, ySpeed, turningSpeed, mSwerveSubsystem.getHeadingRotation2d());
    } else {
      // Relative to robot
      chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, turningSpeed);
    }

    
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
    return false;
  }
}
