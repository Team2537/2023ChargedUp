package frc.robot.swervedrive;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

public class DriveMotor {
  private final CANSparkMax mDriveMotor;
  private final RelativeEncoder mDriveEncoder;

  public DriveMotor(int driveMotorId, boolean driveMotorReversed) {
    mDriveMotor = new CANSparkMax(driveMotorId, MotorType.kBrushless);
    mDriveEncoder = mDriveMotor.getEncoder();
    mDriveMotor.setInverted(driveMotorReversed);

    mDriveMotor.getPIDController().setP(0.2 * 1.0); // if its jerking then kP is probably too big
    mDriveMotor.getPIDController().setI(0.0);
    mDriveMotor.getPIDController().setD(0.0);
    mDriveMotor.getPIDController().setFF(0.0);
    mDriveEncoder.setPosition(0.0);
    mDriveEncoder.setPositionConversionFactor(0.102 * Math.PI / 8.14);// gets distance traveled meters
    mDriveEncoder.setVelocityConversionFactor(0.102 * Math.PI / (60.0 * 8.14)); // 0.102*Math.PI/(8.14*60)

  }

  public double getVelocity() {
    return mDriveEncoder.getVelocity();
  }

  public void setDesiredState(double setVelocityMps) {
    mDriveMotor.getPIDController().setReference(setVelocityMps, CANSparkMax.ControlType.kVelocity);
  }
}