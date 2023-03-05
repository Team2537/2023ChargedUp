// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.Timer;
import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;

/** An example command that uses an example subsystem. */
public class PathCommand extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final SwerveSubsystem mSwerveSubsystem;
  private final PIDController pidController;
  private PathPlannerTrajectory trajectory;
  private Timer timer;

  //private final double kp=0.1, ki=0.0, kd=0.0; //turn smoothly but oscillates at setpoint
  //private final double kp=0.03, ki=0.1, kd=0.0; //turn smoothly but oscillates at setpoint
  //private final double kp=0.01, ki=0.0, kd=0.0; //barely turns
  //private final double kp=0.05, ki=0.0, kd=0.0; //turn smoothly but oscillates at setpoint
  //private final double kp=0.05, ki=0.0, kd=0.01; //turn smoothly but oscillates at setpoint
  private final double kp=1.5, ki=0.1, kd=0.0;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public PathCommand(SwerveSubsystem swerveSubsystem, PathPlannerTrajectory trajectory) {
    mSwerveSubsystem = swerveSubsystem;
    pidController=new PIDController(kp, ki, kd);
    pidController.enableContinuousInput(0,360);

    ShuffleboardTab tab = Shuffleboard.getTab("Swerve State");

    // tab.addNumber("turning speed", () -> turningSpeed);
    // tab.addNumber("desired heading", () -> heading.get());
    // tab.addNumber("our heading", () -> swerveSubsystem.getHeading());

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(swerveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  double turningSpeed = 0;

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    double time = timer.get();
    PathPlannerState desiredState = (PathPlannerState) trajectory.sample(time);
    
    Pose2d desiredPose = desiredState.poseMeters;
    Rotation2d desiredHeading = desiredState.holonomicRotation;
    Pose2d currentPose = mSwerveSubsystem.getPose();
    turningSpeed = pidController.calculate(mSwerveSubsystem.getHeading(), desiredHeading.getDegrees());
    //SmartDashboard.putNumber("Turning speed", turningSpeed);
    double xCurrent = currentPose.getX();
    double xDesired = desiredPose.getX();
    double yCurrent = currentPose.getY();
    double yDesired = desiredPose.getY();

    double x = xDesired-xCurrent;
    double y = yDesired-yCurrent;
    double angleToDesired = Math.tan(y/x); //not sure on coordinate system


    double xSpeed = desiredState.velocityMetersPerSecond*Math.cos(angleToDesired);
    double ySpeed = desiredState.velocityMetersPerSecond*Math.sin(angleToDesired);
    

    ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
      xSpeed, ySpeed, turningSpeed, mSwerveSubsystem.getRotation2d());
    
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