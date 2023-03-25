package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmPivotSubsystem;
import frc.robot.subsystems.ArmTelescopeSubsystem;

import static frc.robot.Constants.ArmConstants.*;

/**
 * This command will reset the arm's position and angle to their origional positions, as well as make sure that the encoder values are correct
 * this uses magnet sensors on the arm thet will send a boolean set to true when they are close
 */
public class HomingCommand extends CommandBase{

    private final ArmPivotSubsystem m_pivotSubsystem;
    private final ArmTelescopeSubsystem m_telescopeSubsystem;

    private boolean retracted = false;

    // Constructor that asigns member variables to passed in subsystems, as well as adds the command requirements
    public HomingCommand(ArmPivotSubsystem pivotSubsystem, ArmTelescopeSubsystem telescopeSubsystem){
        m_pivotSubsystem = pivotSubsystem;
        m_telescopeSubsystem = telescopeSubsystem;

        addRequirements(pivotSubsystem, telescopeSubsystem);
    }
  
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (!retracted) {
            if (m_telescopeSubsystem.getMagnetClosed()) {
                retracted = true;
                m_telescopeSubsystem.reset();
            } else {
                m_telescopeSubsystem.incrementPosition(false, 0.25);
            }
        } /*else if (!m_telescopeSubsystem.isClose(0.4)) {
            m_telescopeSubsystem.incrementPosition(0.4, 0.25);
        }*/ else if (!m_pivotSubsystem.isClose(HOME_ANGLE)) {            
            //m_pivotSubsystem.syncEncoders();
            m_pivotSubsystem.setAngle(HOME_ANGLE);
        }
    }
  
    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        retracted = false;
        // When the command ends, both subsystems are reset
        //m_telescopeSubsystem.reset();
        //m_pivotSubsystem.reset();
        m_pivotSubsystem.setEncoderPosition(HOME_ANGLE);
    }
  
    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return (m_pivotSubsystem.getMagnetClosed() || m_pivotSubsystem.isClose(0.4)) && m_pivotSubsystem.isClose(HOME_ANGLE);
    }

}
