package frc.robot.commands;

import frc.robot.subsystems.GripperSubsystem;
import edu.wpi.first.wpilibj2.command.*;
public class CloseGripperCommand extends CommandBase {
    private GripperSubsystem m_gripperSubsystem;

    public CloseGripperCommand(GripperSubsystem subsystem) {
    m_gripperSubsystem = subsystem;
    addRequirements(m_gripperSubsystem);
    }

    @Override
    public boolean isFinished() {
        m_gripperSubsystem.closeGripper();
        return true;
    }
}
