// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.SwerveSubsystem;

public class BalanceCommand extends CommandBase {
    double target = 0;
    double kP = 0.004; //maybe adjust value. goes too far over when trying to balance
    double kI = 0.0005;
    double kD = 0;
    double e = 0;
    double ePrev = 0;

    double iAccumulator = 0;

    private int count = 0;
    private boolean isEnd = false;
    
    private SwerveSubsystem mSwerveSubsystem;

    private double mPigeonPitch;

    /**
     * This function is run when the robot is first started up and should be used
     * for any
     * initialization code.
     */

    public BalanceCommand(SwerveSubsystem swerveSubsystem, double target) {

        // example comment (showing bobby git)
        addRequirements(swerveSubsystem);
        mSwerveSubsystem = swerveSubsystem;
        this.target = target;
        
    }

    //BalanceCommand
    public BalanceCommand(SwerveSubsystem swerveSubsystem) {
        this(swerveSubsystem, 0.0);
    }

    @Override
    public void initialize() {
        iAccumulator = 0;
    }

    @Override
    public void execute() {
        mPigeonPitch = mSwerveSubsystem.getPitchRotation2d().getDegrees();
        double e = target - mPigeonPitch;

        iAccumulator += 0.02 * e;

        double p = kP * e;
        double i = kI * iAccumulator;
        double d = kD * (e - ePrev) / 0.02;

        ePrev = e;

        double xSpeed = -(p + i + d);
        xSpeed = Math.max(-0.7, Math.min(xSpeed, 0.7));

        ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
            xSpeed, 0.0, 0.0, mSwerveSubsystem.getHeadingRotation2d());
          
           // 5. Convert chassis speeds to individual module states
           SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
      
           // 6. Output each module states to wheels
           mSwerveSubsystem.setModuleStates(moduleStates);


        if(count<25) {
            if(Math.abs(mPigeonPitch)<5.0) {
                count++;
            }
            else {
                count=0;
            }
        } else {
            isEnd = true;
        }


    }

    @Override
    public void end(boolean interrupted) {}
  
    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
   
      return isEnd;
    }
  }