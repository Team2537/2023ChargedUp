package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class ReturnCommand extends SequentialCommandGroup {
    public ReturnCommand(FixedExtensionCommand extension, FixedAngleCommand angle) {
        addCommands(extension, angle);
    }
}
