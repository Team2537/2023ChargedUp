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

public class GripperSubsystem extends SubsystemBase {
  private DigitalInput m_read;
  private boolean opened;

  public GripperSubsystem(double target_low, double target_high, int lidar_read_port, int lidar_trigger_port) {
    m_read = new DigitalInput(4);
    Shuffleboard.getTab("Gripper Subsystem").addBoolean("Gamepiece detected", () -> isTarget());

    closeGripper();
    opened = false;
  }
  
  private final DoubleSolenoid m_solenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 2, 3);


  public void openGripper() {
    m_solenoid.set(kForward);
    opened = true;
  }

  public void closeGripper() {
    m_solenoid.set(kReverse);
    opened = false;
  }

  public boolean isOpened(){
    return opened;
  }

  // implement a low pass filter
  private double lowPass(double raw, double filtered, double alpha) {
    return (1 - alpha) * filtered + alpha * (raw);
  }

  // return true if the target is in target bounds
  public boolean isTarget() {
    return !m_read.get();
  }
}
