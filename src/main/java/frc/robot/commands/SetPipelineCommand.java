package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.VisionSubsystem;


public class SetPipelineCommand extends CommandBase {

    private VisionSubsystem m_subsystem;
    private Integer m_pipeline;

    public SetPipelineCommand(VisionSubsystem subsystem, Integer pipeline){
        m_subsystem = subsystem;
        m_pipeline = pipeline;

        addRequirements(subsystem);
    }

    @Override
    public boolean isFinished() {
        m_subsystem.setPipeline(m_pipeline);
        return true;
    }
}
