package frc.robot.swervedrive;

import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.SensorTimeBase;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class SteerMotor{
private final CANSparkMax mSteerMotor;
  private final RelativeEncoder mSteerEncoder;
  private final CANCoder mAbsoluteEncoder;
  private double mAbsoluteEncoderOffsetRad;

  public SteerMotor(int steerMotorId, int absoluteEncoderId, boolean steerMotorReversed, double absoluteEncoderOffsetRad) {
    mSteerMotor = new CANSparkMax(steerMotorId, MotorType.kBrushless);
    mSteerEncoder = mSteerMotor.getEncoder();
    mAbsoluteEncoder = new CANCoder(absoluteEncoderId);
    mSteerMotor.setInverted(steerMotorReversed);
    mAbsoluteEncoderOffsetRad = absoluteEncoderOffsetRad;
    
    mSteerMotor.getPIDController().setP(0.6);
    mSteerMotor.getPIDController().setI(0.0);
    mSteerMotor.getPIDController().setD(0.0);
    mSteerMotor.getPIDController().setFF(0.0);

   mSteerEncoder.setPosition(mAbsoluteEncoder.getAbsolutePosition()-mAbsoluteEncoderOffsetRad);
   mAbsoluteEncoder.configFeedbackCoefficient(2.0*Math.PI/4096.0, "rad", SensorTimeBase.PerSecond);
   mSteerEncoder.setPositionConversionFactor(2.0*Math.PI/(150.0/7.0));
  }

  public double getPosition() {
    return mSteerEncoder.getPosition();
  }

  public void setDesiredState(double setAngleRad) {
    mSteerMotor.getPIDController().setReference(setAngleRad, CANSparkMax.ControlType.kPosition);
  }
}