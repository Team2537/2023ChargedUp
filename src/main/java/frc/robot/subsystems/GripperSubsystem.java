// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

// import lidar tools
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.*;

/**
 * The GripperSubsystem class is the subsystem that controls the gripper, and relevant sensors
 */
public class GripperSubsystem extends SubsystemBase {
  private DigitalInput m_read;
  private boolean opened;

  /**
   * Constructs the GripperSubsystem class
   * Initializes the IR sensor as well as closing the gripper when the robot starts.
   */
  public GripperSubsystem() {
    m_read = new DigitalInput(4);
    Shuffleboard.getTab("Gripper Subsystem").addBoolean("Gamepiece detected", () -> isTarget());

    closeGripper();
    opened = false;
  }
  
  private final DoubleSolenoid m_solenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 2, 3);


  /**
   * Sends a command to open the gripper
   */
  public void openGripper() {
    m_solenoid.set(kForward);
    opened = true;
  }

  /**
   * Sends a command to close the gripper
   */
  public void closeGripper() {
    m_solenoid.set(kReverse);
    opened = false;
  }

  /**
   * Check if the gripper is opened
   * @return If the gripper is currently opened
   */
  public boolean isOpened(){
    return opened;
  }

  /**
   * Check if there is a gamepiece within range of the gripper
   * @return if there is a gamepiece within range
   */
  public boolean isTarget() {
    return !m_read.get();
  }
}
