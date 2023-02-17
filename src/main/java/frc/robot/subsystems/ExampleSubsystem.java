// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


/**
 * The ExampleSubsystem class is a simple example of a subsystem.
 */
public class ExampleSubsystem extends SubsystemBase {
  private final DoubleLogEntry logEntry;

  /** Creates a new ExampleSubsystem. */
  public ExampleSubsystem() {
    //log example things
    logEntry = new DoubleLogEntry(DataLogManager.getLog(), "/subsystem_name/encoders");
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    logEntry.append(0);
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
