package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmPivotSubsystem;

public class FixedAngleCommand extends CommandBase {

    private final ArmPivotSubsystem m_subsystem;
    private final double m_targetAngle;


    public FixedAngleCommand(ArmPivotSubsystem subsystem, double targetAngle){
        m_subsystem = subsystem;
        m_targetAngle = targetAngle;

        addRequirements(subsystem);
    }

    @Override
    public boolean isFinished() {
      m_subsystem.setAngle(m_targetAngle);
      return true;
    }
}