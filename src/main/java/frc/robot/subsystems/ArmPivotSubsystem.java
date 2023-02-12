// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Ports;

// import revlib spark max motor
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
// import revlib encoder
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxPIDController.AccelStrategy;

import static frc.robot.constants.Ports.*;

public class ArmPivotSubsystem extends SubsystemBase {
  private final CANSparkMax m_motor;
  private final RelativeEncoder m_motorEncoder;
  private final SparkMaxPIDController m_pidController;
  private final DutyCycleEncoder m_shaftEncoder = new DutyCycleEncoder(0);
  DigitalInput m_pivotMagnet = new DigitalInput(PIVOT_MAGNET_SENSOR);


  private final double kP, kI, kD;

  private double target;

  public ArmPivotSubsystem() {
    // initialize motor
    m_motor = new CANSparkMax(Ports.PIVOT_MOTOR, MotorType.kBrushless);

    m_pidController = m_motor.getPIDController();

    // Encoder object initialized to display position values
    m_motorEncoder = m_motor.getEncoder();

    m_motorEncoder.setPositionConversionFactor(360/200f);
    m_motorEncoder.setVelocityConversionFactor(360/200f/60);

    m_motorEncoder.setPosition(m_shaftEncoder.getAbsolutePosition());

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

    // Shuffleboard displayed variables for testing
    Shuffleboard.getTab("Pivoting Arm Subsystem").addBoolean("Homed", () -> getMagnetClosed());
  }

  // Sets the target of the PID control loop to the given angle, with 0 being parallel to the ground
  public void setAngle(double angleDeg) {
    target = angleDeg;
  }
  
  // Sets a raw RPM value for the motor (Overrides the PID controls)
  public void setRawSpeed(double speed){
    m_motor.set(speed);
  }

  // Returns true if the magnet sensors are withing ~2cm of each other
  public boolean getMagnetClosed(){
    return !m_pivotMagnet.get();
  }

  // Runs once per scheduler run (every 20ms)
  @Override
  public void periodic() {
    // Sets the target of the PID loop to the "target" double, with the smart motion control type (Idk what that does)
     m_pidController.setReference(target, ControlType.kSmartMotion);
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
    // Nobody likes this method - falon
  }
}
