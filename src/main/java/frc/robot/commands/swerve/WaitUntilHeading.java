package frc.robot.commands.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveSubsystem;

public class WaitUntilHeading extends CommandBase{

    private final SwerveSubsystem drivebase;
    private final double heading;

    public WaitUntilHeading(SwerveSubsystem drivebase, double heading){
        this.drivebase = drivebase;
        this.heading = heading;
    }

    @Override
    public boolean isFinished() {
        boolean finished = drivebase.getHeading().getDegrees() < 3.0 ? true : false; 
        return finished;
    }
}
