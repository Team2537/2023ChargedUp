package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.GripperSubsystem;

public class AutoGrabCommand extends CommandBase {

    private final GripperSubsystem m_subsystem;

    public AutoGrabCommand(GripperSubsystem subsystem) {
        m_subsystem = subsystem;

        addRequirements(subsystem);
    }

    @Override
    public void initialize(){
        m_subsystem.openGripper();
    }

    @Override
    public void execute() {
        if (m_subsystem.isTarget() && m_subsystem.isOpened()) {
            m_subsystem.closeGripper();
        }
    }
    
    @Override
    public boolean isFinished() {
        return false; // command will end when button is released
    }
}
