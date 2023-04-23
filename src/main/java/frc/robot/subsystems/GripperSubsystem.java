// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.function.Consumer;

// import lidar tools
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.*;

public class GripperSubsystem extends SubsystemBase {
  private static final double FILTERED_GAIN = 0.07;
  private DigitalOutput m_trig;
  private double m_target_high;
  private double m_target_low;
  private DigitalInput m_read;
  private DutyCycle m_pwm;
  private boolean m_vibrate;
  private boolean opened;

  public GripperSubsystem(double target_low, double target_high, int lidar_read_port, int lidar_trigger_port) {
    // init lidar
    m_target_low = target_low;
    m_target_high = target_high;
    /*
     * m_trig = DIOJNI.initializeDIOPort(HAL.getPort((byte) 0), true);
     * DIOJNI.setDIODirection(m_trig, false);
     * DIOJNI.setDIO(m_trig,false);
     */

    // m_trig = new DigitalOutput(lidar_trigger_port);
    // m_trig.set(false);

    /*
     * m_read = DIOJNI.initializeDIOPort(HAL.getPort((byte) 1), true);
     * DIOJNI.setDIODirection(m_read, true);
     */
    m_read = new DigitalInput(4);
    //m_pwm = new DutyCycle(m_read);
    Shuffleboard.getTab("Gripper Subsystem").addBoolean("Gamepiece detected", () -> isTarget());

    //Shuffleboard.getTab("Gripper Subsystem").addNumber("pulse_width_cm_filtered", () -> filtered_pulse_width);
    //Shuffleboard.getTab("Gripper Subsystem").addNumber("pulse_width_cm", () -> m_pwm.getHighTimeNanoseconds() * 10e-5);

    closeGripper();
    opened = false;
  }

  private int m_count;
  private double filtered_pulse_width;
  
  // replace 0 with import from constants package
  
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

  // @Override
  // public void periodic() {
  //   // // get and output the pulse width of m_read
  //   // // System.out.println(DIOJNI.getDIO(m_read));
  //   // boolean inTarget = isTarget();
  //   // // only trigger the sensor every 10th time
  //   // if (m_count++ % 10 == 0 && m_vibrate) {
  //   //   m_rumble.accept(inTarget ? 1.0 / 4.0 : 0.0);
  //   // } else if (m_count % 10 == 0) {
  //   //   m_rumble.accept(0.0);
  //   // }
    
  // }

  // implement a low pass filter
  private double lowPass(double raw, double filtered, double alpha) {
    return (1 - alpha) * filtered + alpha * (raw);
  }

  // return true if the target is in target bounds
  public boolean isTarget() {

    // double pulse_width = m_pwm.getHighTimeNanoseconds() * 10e-5; // should be in cm

    // filtered_pulse_width = lowPass(pulse_width, filtered_pulse_width, FILTERED_GAIN);
    // return filtered_pulse_width > m_target_low && filtered_pulse_width < m_target_high;
    return !m_read.get();

  }

  @Override
  public void periodic() {
    
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
