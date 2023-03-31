// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.SwerveSubsystem;

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
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;

/** An example command that uses an example subsystem. */
public class RotateCommand extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final SwerveSubsystem mSwerveSubsystem;
  private final PIDController pidController;
  private PathPlannerTrajectory trajectory;
  private Timer timer;
  private PIDController xPosController, yPosController;
  private Pose2d startPose2d = new Pose2d(), endPose2d;
  private double endTime;
  private Rotation2d endHeading;
  private boolean isTimeEnd = false;
  private double distanceToEnd = 1000;
  private double angleToEnd = 1000;
  private PathPlannerState startState, endState;

 

  //private final double kp=0.1, ki=0.0, kd=0.0; //turn smoothly but oscillates at setpoint
  //private final double kp=0.03, ki=0.1, kd=0.0; //turn smoothly but oscillates at setpoint
  //private final double kp=0.01, ki=0.0, kd=0.0; //barely turns
  //private final double kp=0.05, ki=0.0, kd=0.0; //turn smoothly but oscillates at setpoint
  //private final double kp=0.05, ki=0.0, kd=0.01; //turn smoothly but oscillates at setpoint
  private final static double kp=0.06, ki=0.0, kd=0.0;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public RotateCommand(SwerveSubsystem swerveSubsystem, Rotation2d endRotation2d) {
    mSwerveSubsystem = swerveSubsystem;
    pidController=new PIDController(kp, ki, kd);
    pidController.enableContinuousInput(0,360);
    endHeading = endRotation2d;
    ShuffleboardTab tab = Shuffleboard.getTab("Swerve State");
    
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(swerveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }


  public static Rotation2d optimize(Rotation2d currentAngle, Rotation2d desiredAngle) {
    Rotation2d delta = currentAngle.minus(desiredAngle);
    //for some reason current-desired works but desired-current doesn't
    if (Math.abs(delta.getDegrees()) > 180.0) {
      return delta.rotateBy(Rotation2d.fromDegrees(-360.0));
    } else {
      return delta;
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    Rotation2d angleToDesired2d = optimize(mSwerveSubsystem.getHeadingRotation2d(), endHeading);
    
    SmartDashboard.putNumber("Desired heading", endHeading.getDegrees());

  
    double turningSpeed = pidController.calculate(angleToDesired2d.getDegrees(), 0.0);
    SmartDashboard.putNumber("Turning speed", turningSpeed);
    
    

    //Checking if we are close to the end of the path
    angleToEnd = Math.abs(mSwerveSubsystem.getHeading()-endHeading.getDegrees());
   
    ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
      0.0, 0.0, turningSpeed, mSwerveSubsystem.getHeadingRotation2d());
    
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
  

    if(angleToEnd<1) {
      System.out.println("A: Rotate Commmand End");
  
      return true;
    }

    return false;
  }
}