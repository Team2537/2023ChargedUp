// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.util.datalog.BooleanLogEntry;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxPIDController.AccelStrategy;

import static frc.robot.Constants.ArmConstants.*;

/**
 * The ArmPivotSubsystem class is the subsystem that controls the arm pivot motor.
 */
public class ArmPivotSubsystem extends SubsystemBase {
  private final CANSparkMax m_motor;
  private final RelativeEncoder m_motorEncoder;
  private final DutyCycleEncoder m_absoluteEncoder;
  private final SparkMaxPIDController m_pidController;

  private final double kP, kI, kD;

  private double target = DEFAULT_ANGLE;

  /**
   * Constructs the arm pivot subsystem
   * Initializes variables as well as setting initial motor and PID settings
   */
  public ArmPivotSubsystem() {
    m_motor = new CANSparkMax(PIVOT_MOTOR, MotorType.kBrushless);
    m_motor.setSmartCurrentLimit(10, 30);

    m_pidController = m_motor.getPIDController();

    m_absoluteEncoder = new DutyCycleEncoder(ABSOLUTE_ENCODER);
    m_absoluteEncoder.setPositionOffset(ABSOLUTE_OFFSET);

    m_motorEncoder = m_motor.getEncoder();

    // set position conversion factor to convert encoder counts to degrees
    m_motorEncoder.setPositionConversionFactor(360/201.6);
    m_motorEncoder.setPosition(ARM_PIVOT_OFFSET);


    // PID coefficients
    kP = 0.0004;
    kI = 0.00000001;
    kD = 0;

    // set PID coefficients
    m_pidController.setP(kP, 0);
    m_pidController.setP(0.001, 1);
    m_pidController.setI(kI);
    m_pidController.setD(kD);

    // set up motor pid settings
    m_pidController.setSmartMotionMaxAccel(15000, 0);
    m_pidController.setSmartMotionMaxVelocity(5676, 0);
    m_pidController.setSmartMotionMinOutputVelocity(0, 0);
    m_pidController.setSmartMotionAllowedClosedLoopError(0.1, 0);
    m_pidController.setSmartMotionAccelStrategy(AccelStrategy.kTrapezoidal, 0);

    // reset motor to keep tests consistent and to resist pid settings continuing from previous runs
    //m_motor.burnFlash();

    // Setup Shuffleboard
    ShuffleboardTab armPivotTab = Shuffleboard.getTab("Pivoting Arm Subsystem");

    armPivotTab.addBoolean("Homed", () -> getMagnetClosed());
    armPivotTab.addNumber("Target Position", () -> target);
    armPivotTab.addNumber("Current Position", () -> getAngle());
    armPivotTab.addNumber("Angular Velocity", () -> m_motorEncoder.getVelocity());
    armPivotTab.addNumber("Absolute Angle", () -> getAbsolutePosition());
  }

  /**
   * @return the current angle of the arm from the motor encoder
   */
  public double getAngle() {
    return m_motorEncoder.getPosition();
  }

  /**
   * @return the current angle of the arm from the absolute encoder
   */
  public double getAbsolutePosition() {
    return m_absoluteEncoder.get() * 360;
  }

  /**
   * Syncs the motor encoder with the absolute encoder
   */
  public void syncEncoders() {
    setAngle(getAbsolutePosition());
    m_motorEncoder.setPosition(getAbsolutePosition());
  }

  /**
   * Sets the target of the PID control loop to the given angle, with 0 being parallel to the ground
   * @param angleDeg angle to set the target to in degrees
   */
  public void setAngle(double angleDeg) {
    target = angleDeg;
  }

  /**
   * Check if the current angle is within 2% of the target
   * @param target Target angle to check against
   * @return If we're within 2% of the target angle
   */
  public boolean isClose(double target){
    return Math.abs((target - m_motorEncoder.getPosition()) / target) <= 0.02;  
  }
  
  /**
   * Sets a raw value for the motor (Overrides the PID controls)
   * @param speed the percent output (0-1) to set the motor to
   */
  public void setRawSpeed(double speed) {
    m_motor.set(speed);
  }

  public void setEncoderPosition(double pos) {
    m_motorEncoder.setPosition(pos);
  }

  /**
   * Switches motor to using velocity PID
   * @param velocity the velocity to pivot at (degrees/s)
   */
  public void setVelocity(double velocity) {
    m_pidController.setReference(velocity, ControlType.kVelocity, 1);
  }

  /**
   * @return true if the magnet sensors are within ~2cm of each other
   */
  public boolean getMagnetClosed() {
    return true;
  }

  @Override
  public void periodic() {
    // Sets the target of the PID loop to the "target" double,
    // using smart motion to control velocity and acceleration while in motion
    m_pidController.setReference(target, ControlType.kSmartMotion);
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
    // Nobody likes this method - falon
    // Factual - micah
    // fr - matthew
  }
}
