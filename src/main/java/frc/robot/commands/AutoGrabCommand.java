package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.GripperSubsystem;

public class AutoGrabCommand extends CommandBase {

    private final GripperSubsystem mGripperSubsystem;

    public AutoGrabCommand(GripperSubsystem subsystem) {
        mGripperSubsystem = subsystem;

        addRequirements(subsystem);
    }

    @Override
    public void initialize(){
        mGripperSubsystem.openGripper();
    }

    @Override
    public void execute() {
       
    }
    
    @Override
    public boolean isFinished() {
        if (mGripperSubsystem.isTarget() && mGripperSubsystem.isOpened()) {
            mGripperSubsystem.closeGripper();
           return true;
        }
    
        return false; // command will end when button is released
    }
}
