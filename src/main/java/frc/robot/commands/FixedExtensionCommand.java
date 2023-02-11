// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.ArmTelescopeSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class FixedExtensionCommand extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final ArmTelescopeSubsystem m_subsystem;

  private final double m_targetExtension;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public FixedExtensionCommand(ArmTelescopeSubsystem subsystem, double targetExtension) {
    m_subsystem = subsystem;
    m_targetExtension = targetExtension;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }


  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //sets the extension to the givin evtension
    m_subsystem.setExtension(m_targetExtension);
    return true;
  }
}
