package frc.robot.subsystems;

import com.ctre.phoenix.sensors.WPI_Pigeon2;
import com.ctre.phoenix.sensors.Pigeon2.AxisDirection;
import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.swerve.TeleopDrive;
import swervelib.SwerveController;
import swervelib.SwerveDrive;
import swervelib.parser.SwerveDriveConfiguration;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;

import java.io.File;
import java.nio.file.Path;
import java.util.Map;
import java.util.List;

public class SwerveSubsystem extends SubsystemBase {

    //swerve drive object
    private final SwerveDrive swerveDrive;
    private SwerveAutoBuilder autoBuilder = null;
    private final CommandXboxController driverXbox = new CommandXboxController(0);
    private ShuffleboardTab swerveTab;
    private WPI_Pigeon2 pigeon2;

    public SwerveSubsystem(File directory){
        SwerveDriveTelemetry.verbosity = SwerveDriveTelemetry.TelemetryVerbosity.HIGH;
        try {
            swerveDrive = new SwerveParser(directory).createSwerveDrive();
        } catch (Exception e) {
            throw new RuntimeException(e);
        }

        swerveTab = Shuffleboard.getTab("Swerve Tab");
        swerveTab.addNumber("Heading", () -> getHeading().getDegrees());
        swerveTab.addNumber("Pitch", () -> getPitch().getDegrees());

        swerveTab.addNumber("Module 1", () -> getSwerveDriveConfiguration().modules[0].getAbsolutePosition());
        swerveTab.addNumber("Module 2", () -> getSwerveDriveConfiguration().modules[1].getAbsolutePosition());
        swerveTab.addNumber("Module 3", () -> getSwerveDriveConfiguration().modules[2].getAbsolutePosition());
        swerveTab.addNumber("Module 4", () -> getSwerveDriveConfiguration().modules[3].getAbsolutePosition());


        setMotorBrake(false);



    }

    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop){
        swerveDrive.drive(translation, rotation, fieldRelative, isOpenLoop);
    }

    @Override
    public void periodic(){}

    @Override
    public void simulationPeriodic(){}

    public SwerveDriveKinematics getKinematics() {
        return swerveDrive.kinematics;
    }

    public void resetOdometry(Pose2d initialHolonomicPose) {
        swerveDrive.resetOdometry(initialHolonomicPose);
    }

    public Pose2d getPose(){
        return swerveDrive.getPose();
    }

    public void setChassisSpeeds(ChassisSpeeds chassisSpeeds){
        swerveDrive.setChassisSpeeds(chassisSpeeds);
    }

    public void postTrajectory(Trajectory trajectory){
        swerveDrive.postTrajectory(trajectory);
    }

    public void zeroGyro(){
        swerveDrive.zeroGyro();
    }

    public void setMotorBrake (boolean brake){
        swerveDrive.setMotorIdleMode(brake);
    }

    public Rotation2d getHeading(){
        return swerveDrive.getYaw();
    }

    public ChassisSpeeds getTargetSpeeds(double xInput, double yInput, double headingX, double headingY) {
        xInput = Math.pow(xInput, 3);
        yInput = Math.pow(yInput, 3);
        return swerveDrive.swerveController.getTargetSpeeds(xInput, yInput, headingX, headingY, getHeading().getRadians());
    }

    public ChassisSpeeds getTargetSpeeds(double xInput, double yInput, Rotation2d angle) {
        xInput = Math.pow(xInput, 3);
        yInput = Math.pow(yInput, 3);
        return swerveDrive.swerveController.getTargetSpeeds(xInput, yInput, angle.getRadians(), getHeading().getRadians());
    }

    public ChassisSpeeds getFieldVelocity(){
        return swerveDrive.getFieldVelocity();
    }

    public ChassisSpeeds getRobotVelocity(){
        return swerveDrive.getRobotVelocity();
    }

    public SwerveController getSwerveController(){
        return swerveDrive.swerveController;
    }

    public SwerveDriveConfiguration getSwerveDriveConfiguration(){
        return swerveDrive.swerveDriveConfiguration;
    }

    public void lock(){
        swerveDrive.lockPose();
    }

    public Rotation2d getPitch() {
        return swerveDrive.getPitch();
    }

    public Command createPathPlannerCommand(String path, PathConstraints constraints, Map<String, Command> eventMap,
                                            PIDConstants translation, PIDConstants rotation, boolean useAllianceColor) {
        List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup(path, constraints);

        if (autoBuilder == null){
            autoBuilder = new SwerveAutoBuilder(
                    swerveDrive::getPose,
                    swerveDrive::resetOdometry,
                    translation,
                    rotation,
                    swerveDrive::setChassisSpeeds,
                    eventMap,
                    useAllianceColor,
                    this
            );
        }

        return autoBuilder.fullAuto(pathGroup);
    }

    public Command getDefaultCommand(){

        Command defaultCommand = new TeleopDrive(this,
                driverXbox::getLeftX,
                driverXbox::getLeftY,
                () -> -driverXbox.getRightX(),
                () -> driverXbox.getHID().getLeftBumper(),
                false,
                true
        );

        return null;
    }




}
