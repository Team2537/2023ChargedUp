package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmPivotSubsystem;
import frc.robot.subsystems.ArmTelescopeSubsystem;

/**
 * This command will reset the arm's position and angle to their origional positions, as well as make sure that the encoder values are correct
 * this uses magnet sensors on the arm thet will send a boolean set to true when they are close
 */
public class HomingCommand extends CommandBase{

    private final ArmPivotSubsystem m_pivotSubsystem;
    private final ArmTelescopeSubsystem m_telescopeSubsystem;

    // Constructor that asigns member variables to passed in subsystems, as well as adds the command requirements
    public HomingCommand(ArmPivotSubsystem pivotSubsystem, ArmTelescopeSubsystem telescopeSubsystem){
        m_pivotSubsystem = pivotSubsystem;
        m_telescopeSubsystem = telescopeSubsystem;

        addRequirements(pivotSubsystem, telescopeSubsystem);
    }
  
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        /*
         * Checks if the telescope's magnet sensor is active
         * if it isn't set the speed to -0.75, if it is then set telescope to 0
         * and do the same thing with the pivot and pivot magnet sensor
         */
        if(!m_telescopeSubsystem.getMagnetClosed()){
            m_telescopeSubsystem.setRawSpeed(-0.75);
        } else if (!m_pivotSubsystem.getMagnetClosed()){
            m_telescopeSubsystem.setRawSpeed(0);
            m_pivotSubsystem.setRawSpeed(-0.1);
        }
    }
  
    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        // When the command ends, both subsystems are reset
        m_telescopeSubsystem.reset();
        m_pivotSubsystem.reset();
    }
  
    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        // Logic to return true when the command should be finished, when both magnet sensors return true
        return m_pivotSubsystem.getMagnetClosed() && m_telescopeSubsystem.getMagnetClosed();
    }

}
