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
public class PathCommand extends CommandBase {
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
  private final static double kp=0.08, ki=0.0, kd=0.0;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public PathCommand(SwerveSubsystem swerveSubsystem, PathPlannerTrajectory trajectory) {
    mSwerveSubsystem = swerveSubsystem;
    pidController=new PIDController(kp, ki, kd);
    xPosController = new PIDController(4.0, 0.1, 0.0);
    yPosController = new PIDController(4.0, 0.1, 0.0);
    pidController.enableContinuousInput(0,360);
    
    ShuffleboardTab tab = Shuffleboard.getTab("Swerve State");
    timer = new Timer();
    timer.stop();
    timer.reset();
    
    this.trajectory = trajectory;
    startState = trajectory.getInitialState();
    startPose2d = startState.poseMeters;
    // mSwerveSubsystem.resetOdometry(startPose2d);
    // mSwerveSubsystem.setHeading(startState.holonomicRotation.getDegrees());

    endState = trajectory.getEndState();
    endHeading = endState.holonomicRotation;
    endPose2d = endState.poseMeters;
    endTime = endState.timeSeconds;
  

    //tab.addNumber("turning speed", () -> turningSpeed);
    //tab.addNumber("desired heading", () -> trajectory.getstat.get());
    
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(swerveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
    timer.stop();
    timer.reset();
    timer.start();
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
    
    double time = timer.get();
    if(time>endTime) {
      isTimeEnd=true;
    } 
    // double headingOffset = 0.0;
    PathPlannerState desiredState = (PathPlannerState) trajectory.sample(time);
    
    Pose2d desiredPose = desiredState.poseMeters;
    Rotation2d desiredHeading = desiredState.holonomicRotation;
    Pose2d currentPose = mSwerveSubsystem.getPose();

    Rotation2d angleToDesired2d = optimize(currentPose.getRotation(), desiredHeading);
    
    SmartDashboard.putNumber("Desired heading", desiredHeading.getDegrees());
    SmartDashboard.putNumber("Time", timer.get());
  
    // if (mSwerveSubsystem.getHeading() - desiredHeading.getDegrees() > 180.0){
    //   headingOffset = -180;
    // }
    // else if(mSwerveSubsystem.getHeading() - desiredHeading.getDegrees() < -180.0){
    //   headingOffset = 180;
    // }
    double turningSpeed = pidController.calculate(angleToDesired2d.getDegrees(), 0.0);
    SmartDashboard.putNumber("Turning speed", turningSpeed);
    double xCurrent = currentPose.getX();
    double xDesired = desiredPose.getX();
    double yCurrent = currentPose.getY();
    double yDesired = desiredPose.getY();

    double x = xDesired-xCurrent;
    double y = yDesired-yCurrent;
    double angleToDesired = Math.tan(y/x); //degrees

    //Checking if we are close to the end of the path
    angleToEnd = Math.abs(mSwerveSubsystem.getHeading()-endHeading.getDegrees());
    distanceToEnd = endPose2d.getTranslation().getDistance(currentPose.getTranslation());

    SmartDashboard.putNumber("xDesired", xDesired);
    SmartDashboard.putNumber("yDesired", yDesired);

    double xSpeed = desiredState.velocityMetersPerSecond*Math.cos(angleToDesired)+xPosController.calculate(xCurrent, xDesired);
    double ySpeed = desiredState.velocityMetersPerSecond*Math.sin(angleToDesired)+yPosController.calculate(yCurrent, yDesired);
    // double xSpeed = xPosController.calculate(xCurrent, xDesired);
    // double ySpeed = yPosController.calculate(yCurrent, yDesired);

    
    SmartDashboard.putNumber("xSpeed", xSpeed);
    SmartDashboard.putNumber("ySpeed", ySpeed);

    ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
      xSpeed, ySpeed, turningSpeed, mSwerveSubsystem.getHeadingRotation2d());
    
     // 5. Convert chassis speeds to individual module states
     SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);

     // 6. Output each module states to wheels
     mSwerveSubsystem.setModuleStates(moduleStates);

     
     
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    timer.stop();
    timer.reset();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // if(isTimeEnd && distanceToEnd<0.5 && angleToEnd<2) {
      
    //   ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
    //     0.0, 0.0, 0.0, mSwerveSubsystem.getHeadingRotation2d());
      
    //    // 5. Convert chassis speeds to individual module states
    //    SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
  
    //    // 6. Output each module states to wheels
    //    mSwerveSubsystem.setModuleStates(moduleStates);
    //    System.out.println("Path End");
    //    timer.stop();
    //   timer.reset();
    //   System.out.println("Path Command End");
    //    return true;
    // }

    // 

    if(isTimeEnd && angleToEnd<1) {
      System.out.println("A: Path Commmand End");
    //   BufferedWriter writer = new BufferedWriter(new FileWriter("C:\\Users\\Public\\Documents\\output\\robot.log"));
    // writer.write("Path Command End");
    
    // writer.close();
      return true;
    }

    return false;
  }
}