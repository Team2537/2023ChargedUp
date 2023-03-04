package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.GripperSubsystem;

public class ToggleGripperCommand extends CommandBase{

    private boolean openGrip;

    private GripperSubsystem m_subsystem;
    
    public ToggleGripperCommand(GripperSubsystem subsystem){
        m_subsystem = subsystem;
        openGrip = true;
        addRequirements(subsystem);
    }

    @Override
    public boolean isFinished(){
        openGrip = m_subsystem.isOpened();
        if(openGrip == true){
            m_subsystem.closeGripper();
        } else {
            m_subsystem.openGripper();
        }
        return true;
    }
}
