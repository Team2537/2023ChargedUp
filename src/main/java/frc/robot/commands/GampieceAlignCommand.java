package frc.robot.commands;

import java.lang.ModuleLayer.Controller;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class GampieceAlignCommand extends CommandBase {


    private VisionSubsystem m_subsystem;
    private SwerveSubsystem m_swerve;

    private PIDController m_PIDcontroller;

    private double targetOffset = 0.0;

    public GampieceAlignCommand(VisionSubsystem subsystem, SwerveSubsystem swerve_subsystem) {
        // Use addRequirements() here to declare subsystem dependencies.
        m_subsystem = subsystem;
        m_swerve = swerve_subsystem;
        addRequirements(subsystem, swerve_subsystem);

        m_PIDcontroller = new PIDController(0.0, 0.0, 0.0);
    
    }
    
    @Override
    public void initialize() {}
  
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
      
    }
  
    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}
  
    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
      return false;
    }
}
