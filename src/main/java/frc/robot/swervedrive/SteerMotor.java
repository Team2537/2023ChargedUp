package frc.robot.swervedrive;

import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.SensorTimeBase;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

/**
 * Represents the motor steering a swerve module.
 * 
 * @see {@link DriveMotor}, {@link SwerveModule}
 */
public class SteerMotor {
  private final CANSparkMax m_steerMotor;
  private final RelativeEncoder m_steerEncoder;

  private final CANCoder m_absoluteEncoder;
  private double m_absoluteEncoderOffsetRad;

  /**
   * @param steerMotorId The ID of the Spark Max controlling the motor.
   * @param absoluteEncoderId The ID of the absolute encoder connected to the swerve module.
   * @param steerMotorReversed Whether or not the motor should be reversed.
   * @param absoluteEncoderOffsetRad The offset from zero of the swerve module when facing forwards.
   */
  public SteerMotor(int steerMotorId, int absoluteEncoderId, boolean steerMotorReversed,
      double absoluteEncoderOffsetRad) {
    m_steerMotor = new CANSparkMax(steerMotorId, MotorType.kBrushless);
    m_steerEncoder = m_steerMotor.getEncoder();

    m_absoluteEncoder = new CANCoder(absoluteEncoderId);
    m_absoluteEncoderOffsetRad = absoluteEncoderOffsetRad;

    m_steerMotor.setInverted(steerMotorReversed);

    m_steerMotor.getPIDController().setP(0.6);
    m_steerMotor.getPIDController().setI(0.0);
    m_steerMotor.getPIDController().setD(0.0);
    m_steerMotor.getPIDController().setFF(0.0);

    m_steerEncoder.setPosition(m_absoluteEncoder.getAbsolutePosition() - m_absoluteEncoderOffsetRad);
    m_steerEncoder.setPositionConversionFactor(2.0 * Math.PI / (150.0 / 7.0));

    m_absoluteEncoder.configFeedbackCoefficient(2.0 * Math.PI / 4096.0, "rad", SensorTimeBase.PerSecond);
  }

  public double getPosition() {
    return m_steerEncoder.getPosition();
  }

  public void setDesiredState(double setAngleRad) {
    m_steerMotor.getPIDController().setReference(setAngleRad, CANSparkMax.ControlType.kPosition);
  }
}