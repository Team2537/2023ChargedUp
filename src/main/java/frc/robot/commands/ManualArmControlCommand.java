package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmPivotSubsystem;
import frc.robot.subsystems.ArmTelescopeSubsystem;

/**
 * Command that allows manual control of the arm pivot and telescope mechanisms
 */
public class ManualArmControlCommand extends CommandBase {
    private final ArmPivotSubsystem m_pivotSubsystem;
    private final ArmTelescopeSubsystem m_telescopeSubsystem;

    private final DoubleSupplier m_pivotControl, m_telescopeControl;

    private double angleRate = 0.05;
    private double extendRate = 0.05;

    private double angleTarget = 0;
    private double extendTarget = 0;

    public ManualArmControlCommand(ArmPivotSubsystem pivotSubsystem, ArmTelescopeSubsystem telescopeSubsystem, DoubleSupplier pivotControl, DoubleSupplier telescopeControl) {
        m_pivotSubsystem = pivotSubsystem;
        m_telescopeSubsystem = telescopeSubsystem;

        m_pivotControl = pivotControl;
        m_telescopeControl = telescopeControl;

        addRequirements(pivotSubsystem, telescopeSubsystem);
    }

    @Override
    public void initialize() {
        extendTarget = m_telescopeSubsystem.getPosition();
        angleTarget = m_pivotSubsystem.getAngle();
    }

    @Override
    public void execute() {

        m_telescopeSubsystem.setExtension(extendTarget += extendRate * m_telescopeControl.getAsDouble());

        m_telescopeSubsystem.setRawSpeed(m_telescopeControl.getAsDouble());
        if (m_pivotSubsystem.getAngle() >= 105 && !(m_pivotControl.getAsDouble() > 0)) {
        } else if (m_pivotSubsystem.getAngle() <= 15 && !(m_pivotControl.getAsDouble() < 0)) {
        } else {
            m_pivotSubsystem.setAngle(angleTarget += angleRate * m_pivotControl.getAsDouble());
        }
    }

    @Override
    public void end(boolean interrupted) {
        // Re-enable positional PID on both subsystems targetting the current position
        m_pivotSubsystem.setAngle(m_pivotSubsystem.getAngle());
        m_telescopeSubsystem.setExtension(m_telescopeSubsystem.getPosition());
    }

    @Override
    public boolean isFinished() {
        // Command should not end automatically since it is controlled only by button press
        return false;
    }
}
