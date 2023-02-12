package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;



public class SetPositionCommand extends CommandBase {

    // Constructor that creates a sequence that runs a FixedAngle and FixedExtension command in a sequence, then schedules it
    public SetPositionCommand(FixedAngleCommand angleCommand, FixedExtensionCommand extensionCommand){
        Commands.sequence(angleCommand, extensionCommand).schedule();
    }

    @Override
    public boolean isFinished(){
        return true;
    }

}
