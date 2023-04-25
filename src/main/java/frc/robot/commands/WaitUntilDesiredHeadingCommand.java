// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.SwerveSubsystem;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class WaitUntilDesiredHeadingCommand extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final SwerveSubsystem mSwerveSubsystem;
  private Rotation2d endHeading;


  public WaitUntilDesiredHeadingCommand(SwerveSubsystem swerveSubsystem, Rotation2d endHeading) {
    mSwerveSubsystem = swerveSubsystem;
    this.endHeading = endHeading;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(Math.abs(mSwerveSubsystem.getHeadingRotation2d().minus(endHeading).getDegrees())<3.0) {
        return true;
    }
    return false;
  }
}
