package frc.robot.swervedrive;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

/**
 * Represents a motor driving the wheel of a swerve module.
 * 
 * @see {@link SteerMotor}, {@link SwerveModule}
 */
public class DriveMotor {
  private final CANSparkMax m_driveMotor;
  private final RelativeEncoder m_driveEncoder;

  /**
   * @param driveMotorId The ID of the Spark Max controlling the motor.
   * @param driveMotorReversed Whether or not the motor should be reversed or not.
   */
  public DriveMotor(int driveMotorId, boolean driveMotorReversed) {
    m_driveMotor = new CANSparkMax(driveMotorId, MotorType.kBrushless);
    m_driveEncoder = m_driveMotor.getEncoder();

    m_driveMotor.setInverted(driveMotorReversed);

    m_driveMotor.getPIDController().setP(0.2);
    m_driveMotor.getPIDController().setI(0.0);
    m_driveMotor.getPIDController().setD(0.0);
    m_driveMotor.getPIDController().setFF(0.0);

    m_driveEncoder.setPosition(0.0);
    m_driveEncoder.setPositionConversionFactor(0.102 * Math.PI / 8.14); // gets distance traveled meters
    m_driveEncoder.setVelocityConversionFactor(0.102 * Math.PI / (60.0 * 8.14));
  }

  public double getVelocity() {
    return m_driveEncoder.getVelocity();
  }

  public void setDesiredState(double setVelocityMps) {
    m_driveMotor.getPIDController().setReference(setVelocityMps, CANSparkMax.ControlType.kVelocity);
  }
}