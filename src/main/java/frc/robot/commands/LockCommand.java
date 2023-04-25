// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.SwerveSubsystem;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.InstantCommand;

/** An example command that uses an example subsystem. */
public class LockCommand extends InstantCommand {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
  private final SwerveSubsystem m_swerveSubsystem;

  public LockCommand(SwerveSubsystem subsystem) {
    m_swerveSubsystem = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    SwerveModuleState[] moduleStates = {
        new SwerveModuleState(0.0, new Rotation2d(Math.PI / 4)),
        new SwerveModuleState(0.0, new Rotation2d(-Math.PI / 4)),
        new SwerveModuleState(0.0, new Rotation2d(-Math.PI / 4)),
        new SwerveModuleState(0.0, new Rotation2d(Math.PI / 4)),
    };

    m_swerveSubsystem.setModuleStates(moduleStates);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_swerveSubsystem.getDefaultCommand().schedule();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    return false;
  }
}
