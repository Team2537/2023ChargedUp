package frc.robot.commands;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmPivotSubsystem;
import frc.robot.subsystems.ArmTelescopeSubsystem;


import static frc.robot.constants.Ports.*;
import static frc.robot.constants.Constants.*;

public class HomingCommand extends CommandBase{

    private final ArmPivotSubsystem m_pivot;
    private final ArmTelescopeSubsystem m_telecope;
    DigitalInput telescopeMagnet = new DigitalInput(TELESCOPE_MAGNET_SENSOR);
    DigitalInput pivotMagnet = new DigitalInput(PIVOT_MAGNET_SENSOR);


    public HomingCommand(ArmPivotSubsystem pivotSubsystem, ArmTelescopeSubsystem telescopeSubsystem){
        m_pivot = pivotSubsystem;
        m_telecope = telescopeSubsystem;

        addRequirements(telescopeSubsystem);
        addRequirements(pivotSubsystem);
    }

    @Override
    public void initialize() {}
  
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if(telescopeMagnet.get()){
            m_telecope.setRawSpeed(-0.1);
        } else if (pivotMagnet.get()){
            m_telecope.setRawSpeed(0.0);
            m_pivot.setRawSpeed(-0.1);
        } else {
            m_pivot.setRawSpeed(0.0);
        }
    }
  
    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}
  
    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
      boolean finished = false;
      if (pivotMagnet.get() && telescopeMagnet.get()){
        finished = true;
      }
      return finished;
    }

}
