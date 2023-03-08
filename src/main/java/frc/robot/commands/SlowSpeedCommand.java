package frc.robot.commands;

import static frc.robot.Constants.DriveConstants.*;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class SlowSpeedCommand extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  
  public SlowSpeedCommand() {}

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (kSpeedMultiplier > 0.5) kSpeedMultiplier = 0.25;
    else kSpeedMultiplier = 1.0;
    return true;
  }
}
