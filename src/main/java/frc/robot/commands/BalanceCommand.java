// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveSubsystem;

public class BalanceCommand extends CommandBase {
    double target = 0;
    double kP = 0.02;
    double kI = 0;
    double kD = 0;
    double e = 0;
    double ePrev = 0;

    double iAccumulator = 0;
    
    private SwerveSubsystem m_swerveSubsystem;

    private DoubleSupplier m_pigeon_pitch;

    /**
     * This function is run when the robot is first started up and should be used
     * for any
     * initialization code.
     */

    public BalanceCommand(SwerveSubsystem swerveSubsystem, double target, DoubleSupplier pigeon_pitch) {

        // example comment (showing bobby git)
        addRequirements(swerveSubsystem);
        this.m_swerveSubsystem = swerveSubsystem;
        this.target = target;
        this.m_pigeon_pitch = pigeon_pitch;
    }

    @Override
    public void initialize() {
        iAccumulator = 0;
    }

    @Override
    public void execute() {
        double pitch = m_pigeon_pitch.getAsDouble();
        double e = target - pitch;

        iAccumulator += 0.02 * e;

        double p = kP * e;
        double i = kI * iAccumulator;
        double d = kD * (e - ePrev) / 0.02;

        ePrev = e;

        double pid = p + i + d;

        m_swerveSubsystem.setStates(pid, 0, 0);
    }
  }
