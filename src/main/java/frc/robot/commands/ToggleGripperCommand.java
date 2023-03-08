package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.GripperSubsystem;

public class ToggleGripperCommand extends CommandBase{
    private GripperSubsystem m_subsystem;
    
    public ToggleGripperCommand(GripperSubsystem subsystem){
        m_subsystem = subsystem;
        addRequirements(subsystem);
    }

    @Override
    public boolean isFinished(){
        if(m_subsystem.isOpened()){
            m_subsystem.closeGripper();
        } else {
            m_subsystem.openGripper();
        }
        return true;
    }
}
