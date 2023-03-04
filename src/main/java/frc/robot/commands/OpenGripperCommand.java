package frc.robot.commands;

import frc.robot.subsystems.GripperSubsystem;
import edu.wpi.first.wpilibj2.command.*;
public class OpenGripperCommand extends CommandBase{
    private GripperSubsystem m_gripperSubsystem;

    public OpenGripperCommand(GripperSubsystem subsystem) {
        m_gripperSubsystem = subsystem;
        addRequirements(m_gripperSubsystem);
    }

    @Override
    public boolean isFinished() {
        m_gripperSubsystem.openGripper();
        return true;
    }
}
