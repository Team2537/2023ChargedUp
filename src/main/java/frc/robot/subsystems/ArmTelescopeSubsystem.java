// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Ports;

// import revlib spark max motor
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
// import revlib encoder
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class ArmTelescopeSubsystem extends SubsystemBase {
  private SparkMaxPIDController m_pidController;
  private RelativeEncoder m_encoder;
  private double kP;
  private double kI;
  private double kD;
  private double kIz;
  private double kFF;
  private double kMaxOutput;
  private double kMinOutput;

  private final DigitalInput magnetSensor = new DigitalInput(9);

  public ArmTelescopeSubsystem() {
    // initialize motor
    m_motor = new CANSparkMax(Ports.EXTENSION_MOTOR, MotorType.kBrushless);
    m_motor.getEncoder().setPositionConversionFactor(1/16f);
    pid = m_motor.getPIDController();

    /**
     * The RestoreFactoryDefaults method can be used to reset the configuration
     * parameters
     * in the SPARK MAX to their factory default state. If no argument is passed,
     * these
     * parameters will not persist between power cycles
     */
    //m_motor.restoreFactoryDefaults();

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
    kP = 0.5;
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

    Shuffleboard.getTab("Telescoping Arm").addNumber("error", () -> target - m_encoder.getPosition());
  }

  SparkMaxPIDController pid;
  CANSparkMax m_motor;
  private double target = 0;

  /*
   * Moves arm to position based on angleDef.
   * Turns the motor;
   * adjusting the angle based on a PIDloop which turns off the motor when the
   * angle is close enough to val.
   * 0 degrees is parallel with the ground
   */
  public void setExtension(double amt) {
    // set goal of pid to amt
    target = amt;
  }

  public void setRawSpeed(double speed){
    m_motor.set(speed);
  }

  public void setRawPosition(double pos){
    m_encoder.setPosition(pos);
  }

  public double getPosition() {
    return m_encoder.getPosition();
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
    if (!magnetSensor.get()){
      m_encoder.setPosition(0.0);
      target = 0.0;
    }
    
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
