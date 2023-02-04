// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.function.Function;

// import revlib spark max motor
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
// import revlib encoder
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
// import lidar tools
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycle;

public class GripperSubsystem extends SubsystemBase {
  private static final double FILTERED_GAIN = 0;
  private SparkMaxPIDController m_pidController;
  private RelativeEncoder m_encoder;
  private double kP;
  private double kI;
  private double kD;
  private double kIz;
  private double kFF;
  private double kMaxOutput;
  private double kMinOutput;
  private DigitalOutput m_trig;
  private double m_target_high;
  private double m_target_low;
  private DigitalInput m_read;
  private DutyCycle m_pwm;
  private boolean m_vibrate;
  private final Function<Double, Boolean> m_rumble;

  public GripperSubsystem(int deviceID, double target_low, double target_high, Function<Double, Boolean> rumble, int lidar_read_port, int lidar_trigger_port) {
    // initialize motor
    m_motor = new CANSparkMax(deviceID, MotorType.kBrushless);
    pid = m_motor.getPIDController();
    m_rumble = rumble;

    /**
     * The RestoreFactoryDefaults method can be used to reset the configuration
     * parameters
     * in the SPARK MAX to their factory default state. If no argument is passed,
     * these
     * parameters will not persist between power cycles
     */
    // m_motor.restoreFactoryDefaults();

    /**
     * In order to use PID functionality for a controller, a SparkMaxPIDController
     * object
     * is constructed by calling the getPIDController() method on an existing
     * CANSparkMax object
     */
    m_pidController = m_motor.getPIDController();

    // Encoder object created to display position values
    m_encoder = m_motor.getEncoder();

    // PID coefficients
    kP = 6e-5;
    kI = 0;
    kD = 0;
    kIz = 0;
    kFF = 0.000015;
    kMaxOutput = 1;
    kMinOutput = -1;

    // set PID coefficients
    m_pidController.setP(kP);
    m_pidController.setI(kI);
    m_pidController.setD(kD);
    m_pidController.setIZone(kIz);
    m_pidController.setFF(kFF);
    m_pidController.setOutputRange(kMinOutput, kMaxOutput);

    m_motor.burnFlash();

    // display PID coefficients on SmartDashboard
    SmartDashboard.putNumber("P Gain", kP);
    SmartDashboard.putNumber("I Gain", kI);
    SmartDashboard.putNumber("D Gain", kD);
    SmartDashboard.putNumber("I Zone", kIz);
    SmartDashboard.putNumber("Feed Forward", kFF);
    SmartDashboard.putNumber("Max Output", kMaxOutput);
    SmartDashboard.putNumber("Min Output", kMinOutput);

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
  private double target;
  private int m_count;
  private double filtered_pulse_width;

  /*
   * Moves arm to position based on angleDef.
   * Turns the motor;
   * adjusting the angle based on a PIDloop which turns off the motor when the
   * angle is close enough to val.
   * 0 degrees is parallel with the ground
   */
  public void armRotate(double angleDeg) {
    // set goal of pid to angleDeg
    target = angleDeg;
  }

  @Override
  public void periodic() {
    /**
     * PIDController objects are commanded to a set point using the
     * SetReference() method.
     * 
     * The first parameter is the value of the set point, whose units vary
     * depending on the control type set in the second parameter.
     * 
     * The second parameter is the control type can be set to one of four
     * parameters:
     * com.revrobotics.CANSparkMax.ControlType.kDutyCycle
     * com.revrobotics.CANSparkMax.ControlType.kPosition
     * com.revrobotics.CANSparkMax.ControlType.kVelocity
     * com.revrobotics.CANSparkMax.ControlType.kVoltage
     */
    m_pidController.setReference(target, CANSparkMax.ControlType.kPosition);

    SmartDashboard.putNumber("Target", target);
    SmartDashboard.putNumber("ProcessVariable", m_encoder.getVelocity());

    // get and output the pulse width of m_read
    // System.out.println(DIOJNI.getDIO(m_read));
    boolean inTarget = isTarget();
    // only trigger the sensor every 10th time
    if (m_count++ % 10 == 0 && m_vibrate) {
      m_rumble.apply(inTarget ? 1.0 / 4.0 : 0.0);
    } else if (m_count % 10 == 0) {
      m_rumble.apply(0.0);
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
