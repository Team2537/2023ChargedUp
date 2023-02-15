// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.ArmTelescopeSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * this sets the target arm extension to what we want it to be
 * this will mostly only be used for the set position command 
 */
public class FixedExtensionCommand extends CommandBase {
  private final ArmTelescopeSubsystem m_subsystem;

  private final double m_targetExtension;


  // Constructor that asigns member variables to passed in variables, as well as adds subsystem requirements
  public FixedExtensionCommand(ArmTelescopeSubsystem subsystem, double targetExtension) {
    m_subsystem = subsystem;
    m_targetExtension = targetExtension;
    addRequirements(subsystem);
  }


  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // Sets the PID target in the Telescoping Subsystem to be the passed in variable
    m_subsystem.setExtension(m_targetExtension);
    return true;
  }
}
