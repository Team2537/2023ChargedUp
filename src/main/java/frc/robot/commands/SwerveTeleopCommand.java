package frc.robot.commands;

import java.util.function.Supplier;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.IOConstants;
import frc.robot.subsystems.SwerveSubsystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public class SwerveTeleopCommand extends CommandBase {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final SwerveSubsystem mSwerveSubsystem;
    private final Supplier<Double> xSpdFunction, ySpdFunction, turningSpdFunction;
    private final Supplier<Boolean> fieldOrientedFunction;
    private final SlewRateLimiter xLimiter, yLimiter, turningLimiter;
    private PathPlannerTrajectory trajectory;
    private Timer timer;
    private double xCurrent, yCurrent, xDesired, yDesired;
  
    /**
     * Creates a new ExampleCommand.
     *
     * @param subsystem The subsystem used by this command.
     */
    public SwerveTeleopCommand(SwerveSubsystem swerveSubsystem,  Supplier<Double> xSpdFunction, Supplier<Double> ySpdFunction, Supplier<Double> turningSpdFunction,
            Supplier<Boolean> fieldOrientedFunction, PathPlannerTrajectory trajectory) {
        mSwerveSubsystem = swerveSubsystem;
        this.xSpdFunction = xSpdFunction;
        this.ySpdFunction = ySpdFunction;
        this.turningSpdFunction = turningSpdFunction;
        this.fieldOrientedFunction = fieldOrientedFunction;
        xLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
        yLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
        turningLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAngularAccelerationUnitsPerSecond);
        ShuffleboardTab tab = Shuffleboard.getTab("Swerve State");
        timer = new Timer();
        timer.start();
        this.trajectory = trajectory;

        tab.addNumber("xCurrent", () -> xCurrent);
        tab.addNumber("yCurrent", () -> yCurrent);
        tab.addNumber("xDesired", () -> xDesired);
        tab.addNumber("yDesired", () -> yDesired);


    // tab.addNumber("turning speed", () -> turningSpeed);
    // tab.addNumber("desired heading", () -> heading.get());
    // tab.addNumber("our heading", () -> swerveSubsystem.getHeading());

        // Use addRequirements() here to declare subsystem dependencies.
      addRequirements(mSwerveSubsystem);
    }
  
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}
  
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        // 1. Get real-time joystick inputs
        
        double xSpeed = xSpdFunction.get();
        double ySpeed = ySpdFunction.get();
        double turningSpeed = turningSpdFunction.get();

        double time = timer.get();
        PathPlannerState desiredState = (PathPlannerState) trajectory.sample(time);
        
        Pose2d desiredPose = desiredState.poseMeters;
        Rotation2d desiredHeading = desiredState.holonomicRotation;
        Pose2d currentPose = mSwerveSubsystem.getPose();
       
        //SmartDashboard.putNumber("Turning speed", turningSpeed);
        xCurrent = currentPose.getX();
        xDesired = desiredPose.getX();
        yCurrent = currentPose.getY();
        yDesired = desiredPose.getY();

        // 2. Apply deadband
        xSpeed = Math.abs(xSpeed) > IOConstants.kDeadband ? xSpeed : 0.0;
        ySpeed = Math.abs(ySpeed) > IOConstants.kDeadband ? ySpeed : 0.0;
        turningSpeed = Math.abs(turningSpeed) > IOConstants.kDeadband ? turningSpeed : 0.0;

        // 3. Make the driving smoother
        // xSpeed = xLimiter.calculate(xSpeed) * DriveConstants.kTeleDriveMaxSpeedMps;
        // ySpeed = yLimiter.calculate(ySpeed) * DriveConstants.kTeleDriveMaxSpeedMps;
        // turningSpeed = turningLimiter.calculate(turningSpeed)
        //         * DriveConstants.kTeleAngularMaxSpeedRps;
        xSpeed = xLimiter.calculate(xSpeed * DriveConstants.kTeleDriveMaxSpeedMps);
        ySpeed = yLimiter.calculate(ySpeed * DriveConstants.kTeleDriveMaxSpeedMps);
        turningSpeed = turningLimiter.calculate(turningSpeed * DriveConstants.kTeleAngularMaxSpeedRps);

        // 4. Construct desired chassis speeds
        ChassisSpeeds chassisSpeeds;
        if (fieldOrientedFunction.get()) {
            // Relative to field
            chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                    xSpeed, ySpeed, turningSpeed, mSwerveSubsystem.getRotation2d());
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
    public void end(boolean interrupted) {}
  
    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
      return false;
    }   
}
