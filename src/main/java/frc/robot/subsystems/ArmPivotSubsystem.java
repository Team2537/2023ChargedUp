// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.util.datalog.BooleanLogEntry;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Ports;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxPIDController.AccelStrategy;

import static frc.robot.constants.Ports.*;

/**
 * The ArmPivotSubsystem class is the subsystem that controls the arm pivot motor.
 */
public class ArmPivotSubsystem extends SubsystemBase {

  private static BooleanLogEntry homedLogEntry;
  private static DoubleLogEntry targetAngleLogEntry;
  private static DoubleLogEntry currentAngleLogEntry;

  private final CANSparkMax m_motor;
  private final RelativeEncoder m_motorEncoder;
  private final SparkMaxPIDController m_pidController;
  
  private final DutyCycleEncoder m_shaftEncoder = new DutyCycleEncoder(ABSOLUTE_ENCODER);
  private final DigitalInput m_pivotMagnet = new DigitalInput(PIVOT_MAGNET_SENSOR);

  boolean m_positionPID = true;

  private final double kP, kI, kD;

  private double target;

  public ArmPivotSubsystem() {
    //this will but the values in the log (untested)
    homedLogEntry = new BooleanLogEntry(DataLogManager.getLog(), "/ArmPivotSubsystem/Homing");
    targetAngleLogEntry = new DoubleLogEntry(DataLogManager.getLog(), "/ArmPivotSubsystem/TargetAngle")
    currentAngleLogEntry = new DoubleLogEntry(DataLogManager.getLog(), "/ArmPivotSubsystem/CurrentAngle")
    // initialize motor
    m_motor = new CANSparkMax(PIVOT_MOTOR, MotorType.kBrushless);

    m_pidController = m_motor.getPIDController();

    // Encoder object initialized to display position values
    m_motorEncoder = m_motor.getEncoder();

    m_motorEncoder.setPositionConversionFactor(360/200f);
    m_motorEncoder.setVelocityConversionFactor(360/200f/60);

    m_shaftEncoder.setPositionOffset(0.222);
    m_motorEncoder.setPosition(m_shaftEncoder.get() * 360);

    // PID coefficients
    kP = 0.0001;
    kI = 0;
    kD = 0;

    // set PID coefficients
    m_pidController.setP(kP);
    m_pidController.setI(kI);
    m_pidController.setD(kD);

    m_pidController.setSmartMotionMaxAccel(10000, 0);
    m_pidController.setSmartMotionMaxVelocity(2500, 0);
    m_pidController.setSmartMotionMinOutputVelocity(0, 0);
    m_pidController.setSmartMotionAllowedClosedLoopError(0.1, 0);
    m_pidController.setSmartMotionAccelStrategy(AccelStrategy.kTrapezoidal, 0);

    m_motor.burnFlash();

    ShuffleboardTab armPivotTab = Shuffleboard.getTab("Pivoting Arm Subsystem");

    armPivotTab.addBoolean("Homed", () -> getMagnetClosed());
    armPivotTab.addNumber("Target Position", () -> target);
    armPivotTab.addNumber("Current Position", () -> getAngle());
    armPivotTab.addNumber("Absolute Position", () -> m_shaftEncoder.get());
    armPivotTab.addNumber("Angular Velocity", () -> m_motorEncoder.getVelocity());
  }

  public double getAngle() {
    return m_motorEncoder.getPosition();
  }

  /**
   * Sets the target of the PID control loop to the given angle, with 0 being parallel to the ground
   * @param angleDeg angle to set the target to in degrees
   */
  public void setAngle(double angleDeg) {
    m_positionPID = true;
    target = angleDeg;
  }
  
  /**
   * Sets a raw value for the motor (Overrides the PID controls)
   * @param speed the percent output (0-1) to set the motor to
   */
  public void setRawSpeed(double speed) {
    m_positionPID = false;
    m_motor.set(speed);
  }

  /**
   * Switches motor to using velocity PID
   * @param velocity the velocity to pivot at (degrees/s)
   */
  public void setVelocity(double velocity) {
    m_positionPID = false;
    m_pidController.setReference(velocity, ControlType.kVelocity);
  }

  /**
   * @return true if the magnet sensors are within ~2cm of each other
   */
  public boolean getMagnetClosed() {
    //return !m_pivotMagnet.get(); TODO change later
    return true;
  }

  /**
   * Resets the motor encoder to sync with the absolute encoder
   */
  public void reset() {
    setVelocity(0);

    m_motorEncoder.setPosition(m_shaftEncoder.get());
    setAngle(m_shaftEncoder.get());
  }

  // Runs once per scheduler run (every 20ms)
  @Override
  public void periodic() {
    // Sets the target of the PID loop to the "target" double,
    // using smart motion to control velocity and acceleration while in motion
    if (m_positionPID) m_pidController.setReference(target, ControlType.kSmartMotion);
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
    // Nobody likes this method - falon
    // Factual - micah
  }
}
