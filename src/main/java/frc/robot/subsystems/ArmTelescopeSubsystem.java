// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.util.datalog.BooleanLogEntry;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
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

/**
 * The ArmTelescopeSubsystem class is the subsystem that controls the telescoping motor.
 */
public class ArmTelescopeSubsystem extends SubsystemBase {
//this creates the log entries
  private static BooleanLogEntry retractedLog;
  private static DoubleLogEntry positionInRevolutionsLog;
  private static DoubleLogEntry positionInInchesLog;
  private static DoubleLogEntry velocityLog;


  private SparkMaxPIDController m_pidController;
  private RelativeEncoder m_encoder;
  private final double kPPosition, kIPosition, kDPosition, kIzPosition, kFFPosition;
  //private final double kPVelocity, kIVelocity, kDVelocity, kIzVelocity, kFFVelocity;

  private double kMaxOutput;
  private double kMinOutput;

  // decides if the pid is targeting position or velocity, true for position, false for velocity
  //private boolean positionPID = true;

  CANSparkMax m_motor;

  private double target = 0;


  private final DigitalInput m_telescopeMagnet = new DigitalInput(TELESCOPE_MAGNET_SENSOR);

  public ArmTelescopeSubsystem() {
// this will put the values in the log
  retractedLog = new BooleanLogEntry(DataLogManager.getLog(), "/ArmTelescopeSubsystem/Retracted");
  positionInRevolutionsLog = new DoubleLogEntry(DataLogManager.getLog(), "/ArmTelescopeSubsystem/PositionInRevolutions");
  positionInInchesLog = new DoubleLogEntry(DataLogManager.getLog(), "/ArmTelescopeSubsystem/PositionInInches");
  velocityLog = new DoubleLogEntry(DataLogManager.getLog(), "/ArmTelescopeSubsystem/Velocity");

    // initialize motor
    m_motor = new CANSparkMax(EXTENSION_MOTOR, MotorType.kBrushless);
    m_motor.setInverted(true);

    m_motor.getEncoder().setPositionConversionFactor(1/16f);
    //m_motor.getEncoder().setVelocityConversionFactor(1/16f);

    // PID object created to display PID values
    m_pidController = m_motor.getPIDController();

    // Encoder object created to display position values
    m_encoder = m_motor.getEncoder();

    // PID coefficients
    kPPosition = 0.5;
    kIPosition = 0;
    kDPosition = 0;
    kIzPosition = 0;
    kFFPosition = 0.000015;

    kMaxOutput = 1;
    kMinOutput = -1;

    // set PID coefficients
    m_pidController.setP(kPPosition);
    m_pidController.setI(kIPosition);
    m_pidController.setD(kDPosition);
    m_pidController.setIZone(kIzPosition);
    m_pidController.setFF(kFFPosition);

    m_pidController.setOutputRange(kMinOutput, kMaxOutput);

    m_motor.burnFlash();

    // Shuffleboard variable display for testing

    // Designate Shuffleboard tab
    ShuffleboardTab telescopingTab = Shuffleboard.getTab("Telescoping Arm");


    telescopingTab.addNumber("Position", () -> getPosition());
    //telescopingTab.addNumber("Position (inches)", () -> getPosInches());
    telescopingTab.addNumber("Velocity", () -> m_encoder.getVelocity());
    telescopingTab.addBoolean("Retracted", () -> getMagnetClosed());
    telescopingTab.addNumber("Target", () -> target);
  }

  public void setExtension(double amt) {
    target = amt;
  }

  /*
   * Disables the positional PID temporarily and uses a velocity PID loop
   * Instead of trying to make it to a target position the loop tries to reach a target velocity
   * Micah broke it :( 
   * NOOO I didn't do anything
   */
  /*public void setVelocity(double velocity) {
    // set goal of pid to a velocity value
    //positionPID = false;
    // set reference for pid in velocity mode
    // notice the use of kSmartVelocity not kVelocity
    m_pidController.setReference(velocity, ControlType.kSmartVelocity, 1);
  }*/

  /**
   * Sets a raw speed to the motor in RPM
   * 
   * @param speed the raw speed (0-1) to set to the motor
   */
  public void setRawSpeed(double speed) {
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

  public boolean isClose(double target){
    return Math.abs((target - m_encoder.getPosition()) / target) <= 0.02;  
  }

  /**
   * Stops the motor for telescoping and resets encoder position
   */
  public void reset() {
    //setRawSpeed(0);
    setEncoderPosition(0);
    setExtension(0);
  }

  // Runs every time the scheduler is called (Every 20ms)
  @Override
  public void periodic() {
//this will continually update the logs with the values from the shuffleboard
  retractedLog.append(getMagnetClosed());
  positionInRevolutionsLog.append(getPosition());
  positionInInchesLog.append(getPosInches());
  velocityLog.append(m_encoder.getVelocity());



    m_pidController.setReference(target, CANSparkMax.ControlType.kPosition);
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
    // I still dislike this method - falon
    // same honestly - micah
    // tbh fr - matthew
  }
}
