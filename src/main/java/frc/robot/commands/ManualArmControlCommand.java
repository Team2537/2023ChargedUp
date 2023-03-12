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

    private double angleRate = 0.75;
    private double extendRate = 0.35;

    private double angleTarget;
    private double extendTarget;

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
        extendTarget += (extendRate * m_telescopeControl.getAsDouble());
        angleTarget += (angleRate * m_pivotControl.getAsDouble());

        extendTarget = Math.max(0, Math.min(extendTarget, 9));
        angleTarget = Math.max(-60, Math.min(angleTarget, 40));

        m_telescopeSubsystem.setExtension(extendTarget);
        m_pivotSubsystem.setAngle(angleTarget);
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
