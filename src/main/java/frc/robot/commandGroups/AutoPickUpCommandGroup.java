package frc.robot.commandGroups;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ArmConstants;
import frc.robot.commands.AutoGrabCommand;
import frc.robot.commands.FixedAngleCommand;
import frc.robot.commands.FixedExtensionCommand;
import frc.robot.subsystems.ArmPivotSubsystem;
import frc.robot.subsystems.ArmTelescopeSubsystem;
import frc.robot.subsystems.GripperSubsystem;

/**
 * This command combines the FixedAngleCommand and the FixedExtensionCommand
 * into one command that will accept two values
 * This is for compacting many lines of code into one command that can easily be
 * accesed and changed
 */

public class AutoPickUpCommandGroup extends SequentialCommandGroup {

    // Constructor that creates a sequence of a FixedAngle and FixedExtension
    // command, then schedules it
    public AutoPickUpCommandGroup(ArmTelescopeSubsystem m_armTelescopeSubsystem, ArmPivotSubsystem m_armPivotSubsystem, GripperSubsystem m_gripperSubsystem){
        addCommands(
            new FixedExtensionCommand(m_armTelescopeSubsystem, ArmConstants.GRAB_EXTENSION),
            new FixedAngleCommand(m_armPivotSubsystem, ArmConstants.GRAB_ANGLE),
            new AutoGrabCommand(m_gripperSubsystem),
            new FixedExtensionCommand(m_armTelescopeSubsystem, 0.0)
        );
       
    }

}
