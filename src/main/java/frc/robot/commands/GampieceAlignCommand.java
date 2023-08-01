package frc.robot.commands;

import java.lang.ModuleLayer.Controller;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.VisionSubsystem;

public class GampieceAlignCommand extends CommandBase {


    private VisionSubsystem m_subsystem;

    private PIDController m_PIDcontroller;

    public GampieceAlignCommand(VisionSubsystem subsystem) {
        // Use addRequirements() here to declare subsystem dependencies.
        m_subsystem = subsystem;
        addRequirements(subsystem);

        m_PIDcontroller = new PIDController(0.0, 0.0, 0.0);
    
    }
    
    @Override
    public void initialize() {}
  
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {}
  
    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}
  
    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
      return false;
    }
}
