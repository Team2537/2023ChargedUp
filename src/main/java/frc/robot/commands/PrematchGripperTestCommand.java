package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.subsystems.GripperSubsystem;

public class PrematchGripperTestCommand extends CommandBase{
    private GripperSubsystem m_gripperSubsystem;
    private Timer timer;
    private boolean lidar_done = false;

    public PrematchGripperTestCommand(GripperSubsystem subsystem) {
        m_gripperSubsystem = subsystem;
        addRequirements(m_gripperSubsystem);
    }

    @Override
    public void initialize() {
        // open gripper to make sure it is open when starting test
        m_gripperSubsystem.openGripper();

        timer = new Timer();
        timer.start();
    }

    // run gripper open and close test for 5 seconds and then stop 
    @Override
    public void execute() {
        if (timer.hasElapsed(9)) {
            if (m_gripperSubsystem.isTarget()) {
                lidar_done = true;
            }
        } else if (timer.hasElapsed(6)) {
            m_gripperSubsystem.openGripper();
        } else if (timer.hasElapsed(3)) {
            m_gripperSubsystem.closeGripper();
        }
    }

    @Override
    public boolean isFinished() {
        return lidar_done;
    }
}