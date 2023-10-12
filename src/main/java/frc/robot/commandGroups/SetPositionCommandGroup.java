package frc.robot.commandGroups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.FixedAngleCommand;
import frc.robot.commands.FixedExtensionCommand;
import frc.robot.commands.ReturnCommand;
import frc.robot.subsystems.ArmTelescopeSubsystem;

/**
 * This command combines the FixedAngleCommand and the FixedExtensionCommand into one command that will accept two values
 *  This is for compacting many lines of code into one command that can easily be accesed and changed
 */
public class SetPositionCommandGroup extends SequentialCommandGroup {


    // Constructor that creates a sequence of a FixedAngle and FixedExtension command, then schedules it
    public SetPositionCommandGroup(FixedExtensionCommand returnExtension, FixedAngleCommand angleCommand, FixedExtensionCommand extensionCommand){
        addCommands(returnExtension, angleCommand, extensionCommand);
    }

}
