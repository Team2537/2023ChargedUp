package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.VisionSubsystem;

public class SwitchPipelineCommand extends CommandBase{

    private VisionSubsystem m_subsystem;

    public SwitchPipelineCommand(VisionSubsystem subsystem){
        m_subsystem = subsystem;

        addRequirements(subsystem);
    }

    @Override
    public boolean isFinished() {
        if (m_subsystem.getPipeline() == 0){
            m_subsystem.setPipeline(1);
        } else if (m_subsystem.getPipeline() == 1){
            m_subsystem.setPipeline(0);
        }
        return true;
    }
}
