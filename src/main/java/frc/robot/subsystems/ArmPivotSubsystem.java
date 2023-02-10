// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import revlib spark max motor
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
// import revlib encoder
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxPIDController.AccelStrategy;

public class ArmPivotSubsystem extends SubsystemBase {
  private final CANSparkMax m_motor;
  private final RelativeEncoder m_motorEncoder;
  private final SparkMaxPIDController m_pidController;
  private final DutyCycleEncoder m_shaftEncoder = new DutyCycleEncoder(0);

  private final double kP, kI, kD;

  private double target;

  public ArmPivotSubsystem(int deviceID) {
    // initialize motor
    m_motor = new CANSparkMax(deviceID, MotorType.kBrushless);

    m_pidController = m_motor.getPIDController();

    // Encoder object initialized to display position values
    m_motorEncoder = m_motor.getEncoder();

    m_motorEncoder.setPositionConversionFactor(360/200f);
    m_motorEncoder.setVelocityConversionFactor(360/200f/60);

    m_motorEncoder.setPosition(m_shaftEncoder.getAbsolutePosition());

    // PID coefficients
    kP = 0.01;
    kI = 0;
    kD = 0;

    // set PID coefficients
    m_pidController.setP(kP);
    m_pidController.setI(kI);
    m_pidController.setD(kD);

    m_pidController.setSmartMotionMaxAccel(0.5, 0);
    m_pidController.setSmartMotionMaxVelocity(15, 0);
    m_pidController.setSmartMotionMinOutputVelocity(0, 0);
    m_pidController.setSmartMotionAllowedClosedLoopError(1, 0);
    m_pidController.setSmartMotionAccelStrategy(AccelStrategy.kTrapezoidal, 0);

    m_motor.burnFlash();
  }

  /*
   * Moves arm to position based on angleDef.
   * Turns the motor;
   * adjusting the angle based on a PIDloop which turns off the motor when the
   * angle is close enough to val.
   * 0 degrees is parallel with the ground
   */
  public void setAngle(double angleDeg) {
    // set goal of pid to angleDeg
    target = angleDeg;
  }

  @Override
  public void periodic() {

     m_pidController.setReference(target, ControlType.kSmartMotion);
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
