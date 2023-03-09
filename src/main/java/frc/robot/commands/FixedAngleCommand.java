package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmPivotSubsystem;

/**
 * this sets the target arm angle to what we want it to be
 * this will mostly only be used for the set position command 
 */
public class FixedAngleCommand extends CommandBase {

    private final ArmPivotSubsystem m_subsystem;
    private final double m_targetAngle;


    // Constructor that asigns member variables to the proper subsystem and target angle, as well as adds requirements
    public FixedAngleCommand(ArmPivotSubsystem subsystem, double targetAngle){
        m_subsystem = subsystem;
        m_targetAngle = targetAngle;

        addRequirements(subsystem);
    }

    @Override
    public boolean isFinished() {
      // Sets the target angle of the pivot arm PID system, then returns true to end the command
      m_subsystem.setAngle(m_targetAngle);
      return m_subsystem.isClose(m_targetAngle);
    }
}