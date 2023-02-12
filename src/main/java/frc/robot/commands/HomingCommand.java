package frc.robot.commands;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmPivotSubsystem;
import frc.robot.subsystems.ArmTelescopeSubsystem;

public class HomingCommand extends CommandBase{

    private final ArmPivotSubsystem m_pivotSubsystem;
    private final ArmTelescopeSubsystem m_telescopeSubsystem;

    // Constructor that asigns member variables to passed in subsystems, as well as adds the command requirements
    public HomingCommand(ArmPivotSubsystem pivotSubsystem, ArmTelescopeSubsystem telescopeSubsystem){
        m_pivotSubsystem = pivotSubsystem;
        m_telescopeSubsystem = telescopeSubsystem;

        addRequirements(telescopeSubsystem);
        addRequirements(pivotSubsystem);
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
        // When the command ends, it sets the speed to zero, as well as the target position and encoder value
        m_telescopeSubsystem.setRawSpeed(0);

        m_telescopeSubsystem.setEncoderPosition(0);
        m_telescopeSubsystem.setExtension(0);
    }
  
    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        // Logic to return true when the command should be finished, when both magnet sensors return true
        return m_pivotSubsystem.getMagnetClosed() && m_telescopeSubsystem.getMagnetClosed();
    }

}
