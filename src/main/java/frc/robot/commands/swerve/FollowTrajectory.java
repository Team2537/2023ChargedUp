package frc.robot.commands.swerve;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveSubsystem;

public class FollowTrajectory extends SequentialCommandGroup {
    public FollowTrajectory(SwerveSubsystem drivebase, PathPlannerTrajectory trajectory, boolean resetOdometry) {
        addRequirements(drivebase);

        if(resetOdometry) {
            drivebase.resetOdometry(trajectory.getInitialHolonomicPose());
        }

        addCommands(
                new PPSwerveControllerCommand(
                        trajectory,
                        drivebase::getPose,
                        Constants.Auton.xAutoPID.createPIDController(),
                        Constants.Auton.yAutoPID.createPIDController(),
                        Constants.Auton.angleAutoPID.createPIDController(),
                        drivebase::setChassisSpeeds,
                        drivebase
                            )

                );
    }
}
