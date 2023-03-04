package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.RGBSubsystem;

public class SetColorCommand extends CommandBase{

    private int m_color;
    private RGBSubsystem m_subsystem;

    public SetColorCommand(RGBSubsystem subsystem, int color){
        m_subsystem = subsystem;
        m_color = color;

        addRequirements(subsystem);
    }

    @Override
    public boolean isFinished(){
        m_subsystem.setCommand(m_color);
        return true;
    }
}
