// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.SwerveSubsystem;

public class BalanceCommand extends CommandBase {
    double target = 0;
    double kP = 0.06;
    double kI = 0;
    double kD = 0;
    double e = 0;
    double ePrev = 0;

    double iAccumulator = 0;
    
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

        ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
            xSpeed, 0.0, 0.0, mSwerveSubsystem.getHeadingRotation2d());
          
           // 5. Convert chassis speeds to individual module states
           SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
      
           // 6. Output each module states to wheels
           mSwerveSubsystem.setModuleStates(moduleStates);
    }

    @Override
    public void end(boolean interrupted) {}
  
    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
      return Math.abs(mPigeonPitch)<5.0;
    }
  }