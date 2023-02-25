package frc.robot.subsystems;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.SensorTimeBase;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.geometry.Rotation2d;

public class SwerveModule {

    private final CANSparkMax m_driveMotor;
    private final CANSparkMax m_steerMotor;

    private final RelativeEncoder m_driveEncoder;
    private final RelativeEncoder m_steerEncoder;

    // private final PIDController mSteerPidController;

    private final CANCoder m_absoluteEncoder;
    private final boolean m_absoluteEncoderReversed;
    private final double m_absoluteEncoderOffsetRad;

    private final ShuffleboardTab m_moduleTab;

    public SwerveModule(int driveMotorId, int steerMotorId, boolean driveMotorReversed, boolean steerMotorReversed,
            int absoluteEncoderId, double absoluteEncoderOffset, boolean absoluteEncoderReversed, String name) {
        // absolute encoders
        m_absoluteEncoderOffsetRad = absoluteEncoderOffset;
        m_absoluteEncoderReversed = absoluteEncoderReversed;
        m_absoluteEncoder = new CANCoder(absoluteEncoderId); // default is degrees per second

        //m_absoluteEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);

        m_absoluteEncoder.configFeedbackCoefficient(ModuleConstants.kAbsoluteEncoderCountsPerMin2Rad, "rad",
                SensorTimeBase.PerSecond); // convert to radians per second
        m_absoluteEncoder.configMagnetOffset(0);

        // motors
        m_driveMotor = new CANSparkMax(driveMotorId, MotorType.kBrushless);
        m_steerMotor = new CANSparkMax(steerMotorId, MotorType.kBrushless);

        // inverting motors
        m_driveMotor.setInverted(driveMotorReversed);
        m_steerMotor.setInverted(steerMotorReversed);

        // motor encoders
        m_driveEncoder = m_driveMotor.getEncoder();
        m_steerEncoder = m_steerMotor.getEncoder();
        
        m_steerMotor.getPIDController().setP(ModuleConstants.kPSteer);
        m_steerMotor.getPIDController().setI(ModuleConstants.kISteer);
        m_steerMotor.getPIDController().setD(ModuleConstants.kDSteer);
        m_steerMotor.getPIDController().setFF(ModuleConstants.kFFSteer);

        m_steerMotor.getPIDController().setOutputRange(-Math.PI, Math.PI);

        m_driveMotor.getPIDController().setP(ModuleConstants.kPDrive);
        m_driveMotor.getPIDController().setI(ModuleConstants.kIDrive);
        m_driveMotor.getPIDController().setD(ModuleConstants.kDDrive);
        m_driveMotor.getPIDController().setFF(ModuleConstants.kFFDrive);

        // so that we can work with meters and radians instead of rotations
        m_driveEncoder.setPositionConversionFactor(0.102 * Math.PI / 8.14);// gets distance traveled meters
        m_driveEncoder.setVelocityConversionFactor(0.102 * Math.PI / (60.0 * 8.14)); // 0.102*Math.PI/(8.14*60)

        m_steerEncoder.setPositionConversionFactor(ModuleConstants.kSteerEncoderRot2Rad);
        m_steerEncoder.setVelocityConversionFactor(ModuleConstants.kSteerEncoderRPM2RadPerSec);

        resetEncoders();

        m_moduleTab = Shuffleboard.getTab(name);

        m_moduleTab.addNumber("Absolute Position", () -> getAbsoluteEncoder());
        m_moduleTab.addNumber("Steer Position", () -> getSteerPosition());
        m_moduleTab.addNumber("Drive Velocity", () -> getDriveVelocity());
    }

    public double getDrivePosition() {
        return m_driveEncoder.getPosition();
    }

    public double getSteerPosition() {
        return m_steerEncoder.getPosition();
    }

    public double getDriveVelocity() {
        return m_driveEncoder.getVelocity(); // should be in meters per second
    }
    

    public double getSteerVelocity() {
        return m_steerEncoder.getVelocity();
    }

    public double getAbsoluteEncoder() {
        double angle = m_absoluteEncoder.getAbsolutePosition() - m_absoluteEncoderOffsetRad;
        return angle * (m_absoluteEncoderReversed ? -1.0 : 1.0);
    }

    public void resetEncoders() {
        m_driveEncoder.setPosition(0.0);
        m_steerEncoder.setPosition(getAbsoluteEncoder());
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getSteerPosition()));
    }



    public void setDesiredState(SwerveModuleState state) {
        // if statement allows us to ignore commands that don't have substantial driving
        // velocity
        
         /*if(Math.abs(state.speedMetersPerSecond) < 0.001) {
            m_driveMotor.getPIDController().setReference(0, ControlType.kVelocity);
            m_driveMotor.set(0);
         } */

        // by taking in the desired state and the current angle the wheels are at,
        // change desired state so that the difference between current and desired angle
        // is minimized
        // set motors to desired state
        m_driveMotor.set(state.speedMetersPerSecond / DriveConstants.kPhysicalMaxSpeedMps);
        m_driveMotor.getPIDController().setReference(state.speedMetersPerSecond, ControlType.kVelocity);

        state = SwerveModuleState.optimize(state, getState().angle);
        m_steerMotor.getPIDController().setReference(state.angle.getRadians(), ControlType.kPosition);
        
    }

    public void stop() {
        m_driveMotor.set(0);
        m_steerMotor.getPIDController().setReference(0.0, ControlType.kPosition);
    }
}