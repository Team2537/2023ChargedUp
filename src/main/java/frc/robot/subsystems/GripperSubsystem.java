// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.function.Consumer;

// import revlib spark max motor
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
// import revlib encoder
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
// import lidar tools
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycle;

public class GripperSubsystem extends SubsystemBase {
  private static final double FILTERED_GAIN = 0;
  private DigitalOutput m_trig;
  private double m_target_high;
  private double m_target_low;
  private DigitalInput m_read;
  private DutyCycle m_pwm;
  private boolean m_vibrate;
  private final Consumer<Double> m_rumble;

  public GripperSubsystem(int deviceID, double target_low, double target_high, Consumer<Double> rumble, int lidar_read_port, int lidar_trigger_port) {
    // initialize motor
    m_motor = new CANSparkMax(deviceID, MotorType.kBrushless);
    m_rumble = rumble;

    // init lidar
    m_target_low = target_low;
    m_target_high = target_high;
    /*
     * m_trig = DIOJNI.initializeDIOPort(HAL.getPort((byte) 0), true);
     * DIOJNI.setDIODirection(m_trig, false);
     * DIOJNI.setDIO(m_trig,false);
     */

    m_trig = new DigitalOutput(lidar_trigger_port);
    m_trig.set(false);

    /*
     * m_read = DIOJNI.initializeDIOPort(HAL.getPort((byte) 1), true);
     * DIOJNI.setDIODirection(m_read, true);
     */
    m_read = new DigitalInput(lidar_read_port);
    m_pwm = new DutyCycle(m_read);
  }

  SparkMaxPIDController pid;
  CANSparkMax m_motor;
  private int m_count;
  private double filtered_pulse_width;

  public void openGripper() {
  }

  public void closeGripper() {
  }

  @Override
  public void periodic() {
    // get and output the pulse width of m_read
    // System.out.println(DIOJNI.getDIO(m_read));
    boolean inTarget = isTarget();
    // only trigger the sensor every 10th time
    if (m_count++ % 10 == 0 && m_vibrate) {
      m_rumble.accept(inTarget ? 1.0 / 4.0 : 0.0);
    } else if (m_count % 10 == 0) {
      m_rumble.accept(0.0);
    }
  }

  // implement a low pass filter
  private double lowPass(double raw, double filtered, double alpha) {
    return (1 - alpha) * filtered + alpha * (raw - filtered);
  }

  // return true if the target is in target bounds
  public boolean isTarget() {
    double pulse_width = m_pwm.getHighTimeNanoseconds() * 10e-5;
    filtered_pulse_width = lowPass(pulse_width, filtered_pulse_width, FILTERED_GAIN);
    return filtered_pulse_width > m_target_low && filtered_pulse_width < m_target_high;
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
