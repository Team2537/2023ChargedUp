package frc.robot.subsystems;

import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.SensorTimeBase;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.geometry.Rotation2d;

/**
 * SwerveModule class for implementation into the SwerveSubsystem
 */
public class SwerveModule {
    

    private final CANSparkMax mDriveMotor;
    private final CANSparkMax mSteerMotor;

    private final RelativeEncoder mDriveEncoder;
    private final RelativeEncoder mSteerEncoder;

    // private final PIDController mSteerPidController;

    private final CANCoder mAbsoluteEncoder;
    private final boolean mAbsoluteEncoderReversed;
    private final double mAbsoluteEncoderOffsetRad;

    /**
     * Constructs the SwerveModule class with offsets and motor IDs
     * @param driveMotorId SparkMax ID for drive motor
     * @param steerMotorId SparkMax ID for steer motor
     * @param driveMotorReversed If the drive motor is reversed
     * @param steerMotorReversed If the steer motor is reversed
     * @param absoluteEncoderId Port of the module's absolute encoder
     * @param absoluteEncoderOffset Offset of the modules absolute encoder
     * @param absoluteEncoderReversed If the absolute encoder is reversed
     */
    public SwerveModule(int driveMotorId, int steerMotorId, boolean driveMotorReversed, boolean steerMotorReversed,
            int absoluteEncoderId, double absoluteEncoderOffset, boolean absoluteEncoderReversed) {
        // absolute encoders
        mAbsoluteEncoderOffsetRad = absoluteEncoderOffset;
        mAbsoluteEncoderReversed = absoluteEncoderReversed;
        mAbsoluteEncoder = new CANCoder(absoluteEncoderId); // default is degrees per second

        mAbsoluteEncoder.configFeedbackCoefficient(ModuleConstants.kAbsoluteEncoderCountsPerMin2Rad, "rad",
                SensorTimeBase.PerSecond); // convert to radians per second

        // motors
        mDriveMotor = new CANSparkMax(driveMotorId, MotorType.kBrushless);
        mSteerMotor = new CANSparkMax(steerMotorId, MotorType.kBrushless);
        // inverting motors
        mDriveMotor.setInverted(driveMotorReversed);
        mSteerMotor.setInverted(steerMotorReversed);
        // motor encoders
        mDriveEncoder = mDriveMotor.getEncoder();
        mSteerEncoder = mSteerMotor.getEncoder();
        
        mSteerMotor.getPIDController().setP(ModuleConstants.kPSteer);
        mSteerMotor.getPIDController().setI(ModuleConstants.kISteer);
        mSteerMotor.getPIDController().setD(ModuleConstants.kDSteer);
        mSteerMotor.getPIDController().setFF(ModuleConstants.kFFSteer);

        mSteerMotor.getPIDController().setOutputRange(-Math.PI, Math.PI);

        mDriveMotor.getPIDController().setP(ModuleConstants.kPDrive);
        mDriveMotor.getPIDController().setI(ModuleConstants.kIDrive);
        mDriveMotor.getPIDController().setD(ModuleConstants.kDDrive);
        mDriveMotor.getPIDController().setFF(ModuleConstants.kFFDrive);

        // so that we can work with meters and radians instead of rotations
        mDriveEncoder.setPositionConversionFactor(0.102 * Math.PI / 8.14);// gets distance traveled meters
        mDriveEncoder.setVelocityConversionFactor(0.102 * Math.PI / (60.0 * 8.14)); // 0.102*Math.PI/(8.14*60)

        mSteerEncoder.setPositionConversionFactor(ModuleConstants.kSteerEncoderRot2Rad);
        mSteerEncoder.setVelocityConversionFactor(ModuleConstants.kSteerEncoderRPM2RadPerSec);

        mDriveMotor.setSmartCurrentLimit(30, 40);
        mSteerMotor.setSmartCurrentLimit(10, 40);

        mDriveMotor.setClosedLoopRampRate(10); //seems to be 1/rate in rev hardware client

        mDriveMotor.burnFlash();
        mSteerMotor.burnFlash();

        Shuffleboard.getTab("Swerve Absolute Encoders").addNumber("" + absoluteEncoderId, () -> mAbsoluteEncoder.getAbsolutePosition());
    }

    /**
     * Get the position of the drive wheel
     * @return the position of the drive wheel
     */
    public double getDrivePosition() {
        return mDriveEncoder.getPosition();
    }

    /**
     * Get the position of the steer wheel
     * @return the position of the steer wheel
     */
    public double getSteerPosition() {
        return mSteerEncoder.getPosition();
    }

    /**
     * Get the velocity of the drive wheel
     * @return the velocity of the drive wheel
     */
    public double getDriveVelocity() {
        return mDriveEncoder.getVelocity(); // should be in meters per second
    }

    /**
     * Get the velocity of the steer wheel
     * @return the velocity of the steer wheel
     */
    public double getSteerVelocity() {
        return mSteerEncoder.getVelocity();
    }

    /**
     * Get the position of the steer wheel from the absolute encoder
     * @return the absolute position of the steer wheel
     */
    public double getAbsoluteEncoder() {
        double angle = mAbsoluteEncoder.getAbsolutePosition() - mAbsoluteEncoderOffsetRad;
        return angle * (mAbsoluteEncoderReversed ? -1.0 : 1.0);

    }

    /**
     * Reset the wheel encoders based on the absolute encoder's position
     */
    public void resetEncoders() {
        mDriveEncoder.setPosition(0.0);
        mSteerEncoder.setPosition(getAbsoluteEncoder());
    }

    /**
     * Get the current state of the module
     * @return the {@link SwerveModuleState} of the module
     */
    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getSteerPosition()));
    }

    /**
     * Get the voltage of the drive motor
     * @return the voltage of the drive motor
     */
    public double getDriveVoltage(){
        return mDriveMotor.getBusVoltage();
    }


    /**
     * Set the state of the module
     * @param state the target state of the module
     */
    public void setDesiredState(SwerveModuleState state) {
        // if statement allows us to ignore commands that don't have substantial driving
        // velocity
        
        //  if(Math.abs(state.speedMetersPerSecond) <0.001) {
        //  stop();
        //  return;
        //  }
         
        // by taking in the desired state and the current angle the wheels are at,
        // change desired state so that the difference between current and desired angle
        // is minimized
        // set motors to desired state
        state = SwerveModuleState.optimize(state, getState().angle);
        mDriveMotor.set(Math.signum(state.speedMetersPerSecond) * Math.min(Math.abs(state.speedMetersPerSecond), DriveConstants.kPhysicalMaxSpeedMps));
        mDriveMotor.getPIDController().setReference(state.speedMetersPerSecond, CANSparkMax.ControlType.kVelocity);
        mSteerMotor.getPIDController().setReference(state.angle.getRadians(), CANSparkMax.ControlType.kPosition);
        SmartDashboard.putString("Swerve[" + mDriveMotor.getDeviceId() + "] state", state.toString());
        SmartDashboard.putNumber("Swerve[" + mDriveMotor.getDeviceId() + "] Actual velocity", getDriveVelocity());
    }

    /**
     * Stop the module's motion
     */
    public void stop() {
        mDriveMotor.set(0);
        mSteerMotor.getPIDController().setReference(0.0, CANSparkMax.ControlType.kPosition);
        
    }
}