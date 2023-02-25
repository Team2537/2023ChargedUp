package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveSubsystem;
import static frc.robot.Constants.*;

public class SwerveTestCommand extends CommandBase {

    private final SwerveSubsystem m_subsystem;
    private final DoubleSupplier m_x, m_y, m_omega;

    public SwerveTestCommand(SwerveSubsystem subsystem, DoubleSupplier x, DoubleSupplier y, DoubleSupplier omega) {
        m_subsystem = subsystem;
        
        m_x = x;
        m_y = y;
        m_omega = omega;

        addRequirements(subsystem);
    }

    @Override 
    public void initialize() {}
    
    @Override
    public void execute() {
        double x = m_x.getAsDouble() * DriveConstants.kTeleDriveMaxSpeedMps * 0.25;
        double y = m_y.getAsDouble() * DriveConstants.kTeleDriveMaxSpeedMps * 0.25;
        double omega = m_omega.getAsDouble() * DriveConstants.kTeleopMaxAngularSpeedRps * 0.25;
        ChassisSpeeds speeds = new ChassisSpeeds(x, y, omega);

        SwerveModuleState[] states = DriveConstants.kDriveKinematics.toSwerveModuleStates(speeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(states, DriveConstants.kTeleDriveMaxSpeedMps);

        m_subsystem.setModuleStates(states);
    }
}
