package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.GripperSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class AutonomousAutoGrabCommand extends CommandBase {
    private final SwerveSubsystem drivebase;
    private final GripperSubsystem mGripperSubsystem;
    private Timer timer;
    private double time = 0.0;


    public AutonomousAutoGrabCommand(SwerveSubsystem drivebase, GripperSubsystem gripperSubsystem) {
        this.drivebase = drivebase;
        mGripperSubsystem = gripperSubsystem;

        addRequirements(drivebase);
        addRequirements(mGripperSubsystem);
    }

    @Override
    public void initialize(){
        mGripperSubsystem.openGripper();
        timer = new Timer();
        timer.stop();
        timer.reset();
        timer.start();
    }

    @Override
    public void execute() {
        time = timer.get();
        if(time<2.0) {
            drivebase.drive(new Translation2d(0.25, 0.25), 0, isFinished(), isFinished());
        }
        
     
    }
    
    @Override
    public boolean isFinished() {
        if (time>0.4 && mGripperSubsystem.isTarget() && mGripperSubsystem.isOpened()) {
            mGripperSubsystem.closeGripper();
            return true;
        }
        return false;
       
    }
}
