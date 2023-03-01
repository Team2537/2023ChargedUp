package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ArmPivotSubsystem;
import frc.robot.subsystems.ArmTelescopeSubsystem;

public class ArmTestCommand extends SequentialCommandGroup {

    private ArmPivotSubsystem m_pivotSubsystem;
    private ArmTelescopeSubsystem m_telescopeSubsystem;

    private HomingCommand homingCommand;
    private FixedAngleCommand zero;
    private FixedAngleCommand up45;
    private FixedAngleCommand down45;
    private FixedExtensionCommand out7;
    private FixedExtensionCommand zeroExtend;

    


    public ArmTestCommand(ArmPivotSubsystem pivotSubsystem, ArmTelescopeSubsystem telescopeSubsystem){
        m_pivotSubsystem = pivotSubsystem;
        m_telescopeSubsystem = telescopeSubsystem;

        homingCommand = new HomingCommand(pivotSubsystem, telescopeSubsystem);
        zero = new FixedAngleCommand(pivotSubsystem, 0);
        up45 = new FixedAngleCommand(pivotSubsystem, 45);
        down45 = new FixedAngleCommand(pivotSubsystem, -45);
        out7 = new FixedExtensionCommand(telescopeSubsystem, 7);
        zeroExtend = new FixedExtensionCommand(telescopeSubsystem, 0);

        addRequirements(pivotSubsystem, telescopeSubsystem);
        addCommands(homingCommand, zero, up45, down45, zero, out7, zeroExtend, homingCommand);
    }
    
}
