package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.GripperSubsystem;

/**
 * Command to close the gripper automatically when a gamepiece is within range
 */
public class AutoGrabCommand extends CommandBase {

    private final GripperSubsystem m_gripperSubsystem;
    private final Timer m_delayTimer;

    boolean reset = true;

    public AutoGrabCommand(GripperSubsystem subsystem) {
        m_gripperSubsystem = subsystem;

        m_delayTimer = new Timer();

        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        m_gripperSubsystem.openGripper();
    }

    @Override
    public void execute() {
        if (m_gripperSubsystem.isTarget() && m_gripperSubsystem.isOpened()) {
            if (reset) m_delayTimer.reset();
            m_delayTimer.start();
            reset = false;
        } else {
            m_delayTimer.stop();
            reset = true;
        }

    }

    @Override
    public void end(boolean interrupted) {
        m_delayTimer.stop();
        m_delayTimer.reset();
    }

    @Override
    public boolean isFinished() {
        if (m_delayTimer.hasElapsed(0.15)) {
            m_gripperSubsystem.closeGripper();
            return true;
        }

        return false; // command will toggle on button press
    }
}
