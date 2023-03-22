// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.SwerveSubsystem;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class SetChassisStateCommand extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final SwerveSubsystem mSwerveSubsystem;
  private final Supplier<Double> xSpdFunction, ySpdFunction, headingFunction;
  private final PIDController pidController;

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
  public SetChassisStateCommand(SwerveSubsystem swerveSubsystem,  Supplier<Double> xSpdFunction, Supplier<Double> ySpdFunction, Supplier<Double> heading) {
    mSwerveSubsystem = swerveSubsystem;
    this.xSpdFunction = xSpdFunction; //xspeed we want to go forward(+) backward(-)
    this.ySpdFunction = ySpdFunction; //yspeed left(-) and right(-)
    this.headingFunction = heading; //direction robot's facing
    pidController=new PIDController(kp, ki, kd);
    pidController.enableContinuousInput(0,360); //maximum 360 degrees

    ShuffleboardTab tab = Shuffleboard.getTab("Swerve State");

    tab.addNumber("turning speed", () -> turningSpeed);
    tab.addNumber("desired heading", () -> heading.get());
    

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(swerveSubsystem); //declares what subsystems that are being used for command
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  double turningSpeed = 0;

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double xSpeed = xSpdFunction.get();
    double ySpeed = ySpdFunction.get();
    double heading = headingFunction.get();
    
    turningSpeed = pidController.calculate(mSwerveSubsystem.getHeading(), heading); 
    
    ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds( 
      xSpeed, ySpeed, turningSpeed, mSwerveSubsystem.getHeadingRotation2d());
    
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