package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.GripperSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class AutonomousAutoGrabCommand extends CommandBase {
    private final SwerveSubsystem mSwerveSubsystem;
    private final GripperSubsystem mGripperSubsystem;
    private Timer timer;
    private double time = 0.0;

    public AutonomousAutoGrabCommand(SwerveSubsystem swerveSubsystem, GripperSubsystem gripperSubsystem) {
        mSwerveSubsystem = swerveSubsystem;
        mGripperSubsystem = gripperSubsystem;

        addRequirements(mSwerveSubsystem);
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
            ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
      0.25, 0.0, 0.0, mSwerveSubsystem.getHeadingRotation2d());

      // 5. Convert chassis speeds to individual module states
     SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);

     // 6. Output each module states to wheels
     mSwerveSubsystem.setModuleStates(moduleStates);
        }
        else {
            ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
      0.0, 0.0, 0.0, mSwerveSubsystem.getHeadingRotation2d());

      // 5. Convert chassis speeds to individual module states
     SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);

     // 6. Output each module states to wheels
     mSwerveSubsystem.setModuleStates(moduleStates);
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
