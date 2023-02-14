package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;


/**
 * This command combines the FixedAngleCommand and the FixedExtensionCommand into one command that will accept two values.
 *  This is for compacting many lines of code into one command that can easily be accesed and changed
 */
public class SetPositionCommand extends CommandBase {

    // Constructor that creates a sequence of a FixedAngle and FixedExtension command, then schedules it
    public SetPositionCommand(FixedAngleCommand angleCommand, FixedExtensionCommand extensionCommand){
        Commands.sequence(angleCommand, extensionCommand).schedule();
    }

    @Override
    public boolean isFinished(){
        return true;
    }

}
