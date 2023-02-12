// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import static frc.robot.constants.Ports.*;

public class ArmTelescopeSubsystem extends SubsystemBase {
  private SparkMaxPIDController m_pidController;
  private RelativeEncoder m_encoder;
  private final double kPPosition, kIPosition, kDPosition, kIzPosition, kFFPosition;
  private final double kPVelocity, kIVelocity, kDVelocity, kIzVelocity, kFFVelocity;

  private double kMaxOutput;
  private double kMinOutput;

  private boolean positionPID = true;

  CANSparkMax m_motor;

  private double target = 0;


  private final DigitalInput m_telescopeMagnet = new DigitalInput(TELESCOPE_MAGNET_SENSOR);

  public ArmTelescopeSubsystem() {
    // initialize motor
    m_motor = new CANSparkMax(FRONT_LEFT_DRIVE, MotorType.kBrushless);
    m_motor.setInverted(true);

    m_motor.getEncoder().setPositionConversionFactor(1/16f);
    m_motor.getEncoder().setVelocityConversionFactor(1/16f);

    m_pidController = m_motor.getPIDController();
    m_encoder = m_motor.getEncoder();

    // PID coefficients
    kPPosition = 0.5;
    kIPosition = 0;
    kDPosition = 0;
    kIzPosition = 0;
    kFFPosition = 0.000015;

    kPVelocity = 0.005;
    kIVelocity = 2e-5;
    kDVelocity = 1e-7;
    kIzVelocity = 0;
    kFFVelocity = 0.00001;

    kMaxOutput = 1;
    kMinOutput = -1;

    // set PID coefficients
    m_pidController.setP(kPPosition, 0);
    m_pidController.setI(kIPosition, 0);
    m_pidController.setD(kDPosition, 0);
    m_pidController.setIZone(kIzPosition, 0);
    m_pidController.setFF(kFFPosition, 0);

    m_pidController.setP(kPVelocity, 1);
    m_pidController.setI(kIVelocity, 1);
    m_pidController.setD(kDVelocity, 1);
    m_pidController.setIZone(kIzVelocity, 1);
    m_pidController.setFF(kFFVelocity, 1);

    m_pidController.setOutputRange(kMinOutput, kMaxOutput);

    m_motor.burnFlash();

    // Shuffleboard variable display for testing

    ShuffleboardTab telescopingTab = Shuffleboard.getTab("Telescoping Arm");

    telescopingTab.addNumber("Position (revolutions)", () -> getPosition());
    telescopingTab.addNumber("Position (inches)", () -> getPosInches());
    telescopingTab.addNumber("Velocity", () -> m_encoder.getVelocity());
    telescopingTab.addBoolean("Retracted", () -> getMagnetClosed());
  }

  public void setExtension(double amt) {
    // set goal of pid to amt
    positionPID = true;
    target = amt;
  }

  /*
   * Disables the positional PID temporarily and uses a velocity PID loop
   * Instead of trying to make it to a target position the loop tries to reach a target velocity
   * Micah broke it :( 
   * NOOO I didn't do anything
   */
  public void setVelocity(double velocity) {
    positionPID = false;
    m_pidController.setReference(velocity, ControlType.kSmartVelocity, 1);
  }

  /**
   * Sets a raw speed to the motor in RPM
   * 
   * @param speed the raw speed (0-1) to set to the motor
   */
  public void setRawSpeed(double speed) {
    positionPID = false;
    m_motor.set(speed);
  }

  /**
   * Manually sets the encoder position to a specified value
   * @param pos the position to set the encoder to
   */
  public void setEncoderPosition(double pos){
    m_encoder.setPosition(pos);
  }

  /**
   * @return the encoder value
   */ 
  public double getPosition() {
    return m_encoder.getPosition();
  }

  /**
   * @return the encoder value in inches
   */
  public double getPosInches(){
    return Math.PI*0.75*m_encoder.getPosition();
  }

  /**
   * @return true when the magnets pick each other up
   */
  public boolean getMagnetClosed(){
    return !m_telescopeMagnet.get();
  }

  /**
   * Stops the motor for telescoping and resets encoder position
   */
  public void reset() {
    setRawSpeed(0);
    setEncoderPosition(0);
    setExtension(0);
  }

  // Runs every time the scheduler is called (Every 20ms)
  @Override
  public void periodic() {
    if (positionPID) m_pidController.setReference(target, CANSparkMax.ControlType.kPosition, 0);
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
    // I still dislike this method - falon
    // same honestly - micah
  }
}
