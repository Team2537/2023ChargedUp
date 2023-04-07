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
  private final double m_extensionRate = 0.5;


  // Constructor that asigns member variables to passed in variables, as well as adds subsystem requirements
  public FixedExtensionCommand(ArmTelescopeSubsystem subsystem, double targetExtension) {
    m_subsystem = subsystem;
    m_targetExtension = targetExtension;
    addRequirements(subsystem);
  }

  @Override
  public void execute(){
    // double position = m_subsystem.getPosition() + m_extensionRate * Math.signum(m_targetExtension - m_subsystem.getPosition());
    // if(Math.abs(m_targetExtension - position) < m_extensionRate){
    //   position = m_targetExtension;
    // }
    // m_subsystem.setExtension(position);

    m_subsystem.incrementPosition(m_targetExtension, m_extensionRate);
  }

  @Override
  public void end(boolean interrupted) {
    System.out.println("Finished");
  }
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // Sets the PID target in the Telescoping Subsystem to be the passed in variable
    return m_subsystem.isClose(m_targetExtension);
  }
}
