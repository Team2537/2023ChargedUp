package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmPivotSubsystem;

/**
 * Command to sync the motor encoder in the pivot arm to the absolute encoder
 */
public class ArmEncoderSyncCommand extends CommandBase {
    
    private final ArmPivotSubsystem m_subsystem;

    public ArmEncoderSyncCommand(ArmPivotSubsystem subsystem) {
        m_subsystem = subsystem;

        addRequirements(subsystem);
    }

    @Override
    public boolean isFinished() {
        m_subsystem.syncEncoders();
        return true;
    }
}
