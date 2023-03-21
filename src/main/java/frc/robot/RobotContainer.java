// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.IOConstants;
import frc.robot.Constants.LidarConstants;
import frc.robot.commandGroups.SetPositionCommandGroup;
import frc.robot.subsystems.SwerveSubsystem;
import static frc.robot.Constants.ArmConstants.*;
import static frc.robot.Constants.ColorConstants.*;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.RGBSubsystem;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import static frc.robot.Constants.IOConstants.*;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    public final XboxController m_controller = new XboxController(IOConstants.kXboxControllerPort); //TODO: make private
    private final LogitechJoystick m_gunnerJoystick = new LogitechJoystick(1);

    // The robot's subsystems and commands are defined here...
    private final SwerveSubsystem m_swerveSubsystem = new SwerveSubsystem();
    private final ArmPivotSubsystem m_armPivotSubsystem = new ArmPivotSubsystem();
    private final ArmTelescopeSubsystem m_armTelescopeSubsystem = new ArmTelescopeSubsystem();
    private final GripperSubsystem m_gripperSubsystem = new GripperSubsystem(LidarConstants.kTargetLow,LidarConstants.kTargetHigh, i -> {
    }, LidarConstants.kLidarReadPort, LidarConstants.kLidarTriggerPort);
    private final CameraSubsystem m_cameraSubsystem = new CameraSubsystem();

    private final LockCommand lockCommand = new LockCommand(m_swerveSubsystem);
    private final ZeroHeadingCommand m_zeroHeadingCommand = new ZeroHeadingCommand(m_swerveSubsystem);

    private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
    private final RGBSubsystem m_RgbSubsystem = new RGBSubsystem(1);

    private final FixedAngleCommand m_bottomRowAngle = new FixedAngleCommand(m_armPivotSubsystem, BOTTOM_ROW_ANGLE);
    private final FixedExtensionCommand m_bottomRowExtension = new FixedExtensionCommand(m_armTelescopeSubsystem,
            BOTTOM_ROW_EXTENSION);
    private final SetPositionCommandGroup m_bottomRowPosition = new SetPositionCommandGroup(m_bottomRowAngle,
            m_bottomRowExtension);

    // Position for middle cone row when bumpers are against the community
    private final FixedAngleCommand m_middleRowAngle = new FixedAngleCommand(m_armPivotSubsystem, MIDDLE_ROW_ANGLE);
    private final FixedExtensionCommand m_middleRowExtension = new FixedExtensionCommand(m_armTelescopeSubsystem,
            MIDDLE_ROW_EXTENSION);
    private final SetPositionCommandGroup m_middleRowPosition = new SetPositionCommandGroup(m_middleRowAngle,
            m_middleRowExtension);

    // Position for top cone row when bumpers are against the community
    private final FixedAngleCommand m_topRowAngle = new FixedAngleCommand(m_armPivotSubsystem, TOP_ROW_ANGLE);
    private final FixedExtensionCommand m_topRowExtension = new FixedExtensionCommand(m_armTelescopeSubsystem,
            TOP_ROW_EXTENSION);
    private final SetPositionCommandGroup m_topRowPosition = new SetPositionCommandGroup(m_topRowAngle, m_topRowExtension);


    //positions for picking up from ground
    private final FixedExtensionCommand m_grabExtension = new FixedExtensionCommand(m_armTelescopeSubsystem,
            GRAB_EXTENSION);
    private final FixedAngleCommand m_grabAngle = new FixedAngleCommand(m_armPivotSubsystem, GRAB_ANGLE);
    private final SetPositionCommandGroup m_grabPosition = new SetPositionCommandGroup(m_grabAngle, m_grabExtension);

    private final FixedExtensionCommand m_shelfExtension = new FixedExtensionCommand(m_armTelescopeSubsystem, SHELF_EXTENSION);
    private final FixedAngleCommand m_shelfAngle = new FixedAngleCommand(m_armPivotSubsystem, SHELF_ANGLE);
    private final SetPositionCommandGroup m_shelfPosition = new SetPositionCommandGroup(m_shelfAngle, m_shelfExtension);

    private final FixedExtensionCommand test = new FixedExtensionCommand(m_armTelescopeSubsystem, 5);
    private final HomingCommand m_homingCommand = new HomingCommand(m_armPivotSubsystem, m_armTelescopeSubsystem);

    private final FixedAngleCommand m_zeroAngleCommand = new FixedAngleCommand(m_armPivotSubsystem, 0);
    // private final SetPositionCommand testPosition = new
    // SetPositionCommand(testAngle, test);

    // private final OpenGripperCommand openGripper = new
    // OpenGripperCommand(m_gripperSubsystem);
    // private final CloseGripperCommand closeGripper = new
    // CloseGripperCommand(m_gripperSubsystem);
    private final ToggleGripperCommand toggleGripper = new ToggleGripperCommand(m_gripperSubsystem);

    private final AutoGrabCommand m_autoGrabCommand = new AutoGrabCommand(m_gripperSubsystem);

    private final FixedExtensionCommand m_returnExtension = new FixedExtensionCommand(m_armTelescopeSubsystem, 0);

    private final ManualArmControlCommand m_manualControl = new ManualArmControlCommand(
            m_armPivotSubsystem,
            m_armTelescopeSubsystem,
            () -> m_gunnerJoystick.getAxis(1),
            () -> 0.1 * m_gunnerJoystick.getHatSwitch());

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    private final SetColorCommand redColor = new SetColorCommand(m_RgbSubsystem, RED);
    private final SetColorCommand yellowColor = new SetColorCommand(m_RgbSubsystem, YELLOW);
    private final SetColorCommand purpleColor = new SetColorCommand(m_RgbSubsystem, PURPLE);
    private final SetColorCommand greenColor = new SetColorCommand(m_RgbSubsystem, GREEN);
    private final SetColorCommand awesomeColor = new SetColorCommand(m_RgbSubsystem, AWESOME);

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {

        m_swerveSubsystem.setDefaultCommand(new SwerveTeleopCommand(
                m_swerveSubsystem,
                () -> -m_controller.getLeftY(), // xSpdFunction is for forward direction
                () -> -m_controller.getLeftX(),
                () -> -m_controller.getRightX(),
                () -> !m_controller.getRightBumper(),
                () -> (m_controller.getLeftTriggerAxis() > 0.75 && !(m_controller.getRightTriggerAxis() > 0.75))));

        // Configure the button bindings
        configureButtonBindings();

        ShuffleboardTab controllerTab = Shuffleboard.getTab("Controller");
        controllerTab.addNumber("Left X", () -> m_controller.getLeftX());
        controllerTab.addNumber("Left Y", () -> m_controller.getLeftY());
        controllerTab.addNumber("Right X", () -> m_controller.getRightX());
        controllerTab.addNumber("Right Y", () -> m_controller.getRightY());
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be
     * created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
     * it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        Trigger yButton = new Trigger(() -> m_controller.getYButton());
        // Trigger rightTrigger = new Trigger(() -> controller.getRightTriggerAxis() >
        // 0.9);
        // rightTrigger.onTrue(new LockCommand(swerveSubsystem));

        yButton.toggleOnTrue(lockCommand);

        Trigger bothTriggers = new Trigger(
                () -> (m_controller.getLeftTriggerAxis() > 0.75 && m_controller.getRightTriggerAxis() > 0.75));

     
        bothTriggers.onTrue(m_zeroHeadingCommand);

        m_gunnerJoystick.getButton(8).onTrue(m_homingCommand);
        m_gunnerJoystick.getButton(2).onTrue(toggleGripper);

        // m_gunnerJoystick.getButton(2).onTrue(m_bottomRowPosition);
        m_gunnerJoystick.getButton(12).onTrue(m_zeroAngleCommand);

        m_gunnerJoystick.getButton(11).onTrue(m_bottomRowPosition);
        m_gunnerJoystick.getButton(9).onTrue(m_middleRowPosition);
        m_gunnerJoystick.getButton(7).onTrue(m_topRowPosition);
        m_gunnerJoystick.getButton(4).onTrue(m_grabPosition);
        m_gunnerJoystick.getButton(10).onTrue(m_shelfPosition);

        m_gunnerJoystick.getButton(6).onTrue(m_returnExtension);
        m_gunnerJoystick.getThrottle().whileTrue(m_manualControl);

        m_gunnerJoystick.getButton(5).toggleOnTrue(purpleColor);
        m_gunnerJoystick.getButton(3).toggleOnTrue(yellowColor);

        m_gunnerJoystick.getButton(1).whileTrue(m_autoGrabCommand);
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        
        PathPlannerTrajectory trajectory = PathPlanner.loadPath("CenterBalance",
                new PathConstraints(AutoConstants.kMaxSpeedMps, AutoConstants.kMaxAccelerationMetersPerSecondSquared));
       

         return new SwerveHomingCommand(m_swerveSubsystem).andThen(new PathCommand(m_swerveSubsystem, trajectory).andThen(
                new BalanceCommand(m_swerveSubsystem).andThen(new LockCommand(m_swerveSubsystem))));
        
        //         return new PathCommand(m_swerveSubsystem, trajectory).andThen(
        //        new BalanceCommand(m_swerveSubsystem)).andThen(
        //         new LockCommand(m_swerveSubsystem));

        // return new HomingCommand(m_armPivotSubsystem, m_armTelescopeSubsystem).andThen(
        //         new FixedAngleCommand(m_armPivotSubsystem, 15.66)).andThen(
        //         new FixedExtensionCommand(m_armTelescopeSubsystem, 6.4)).andThen(
        //         new OpenGripperCommand(m_gripperSubsystem)).andThen(
        //         new WaitCommand(0.5)).andThen(
        //         new HomingCommand(m_armPivotSubsystem, m_armTelescopeSubsystem).alongWith(
        //         new PathCommand(m_swerveSubsystem, trajectory)).andThen(
        //         new BalanceCommand(m_swerveSubsystem)).andThen(
        //         new LockCommand(m_swerveSubsystem))) ;

        // return new HomingCommand(m_armPivotSubsystem, m_armTelescopeSubsystem).andThen(
        //         new FixedAngleCommand(m_armPivotSubsystem, 15.66)).andThen(
        //         new FixedExtensionCommand(m_armTelescopeSubsystem, 6.4)).andThen(
        //         new OpenGripperCommand(m_gripperSubsystem)).andThen(
        //         new WaitCommand(0.5)).andThen(
        //         new HomingCommand(m_armPivotSubsystem, m_armTelescopeSubsystem).alongWith(
        //         new PathCommand(m_swerveSubsystem, trajectory)));

        // // 1. Create trajectory settings
        // TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
        // AutoConstants.kMaxSpeedMps,
        // AutoConstants.kMaxAccelerationMetersPerSecondSquared)
        // .setKinematics(DriveConstants.kDriveKinematics);

        // // 2. Generate trajectory
        // Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
        // new Pose2d(0, 0, new Rotation2d(0)),
        // List.of(
        // new Translation2d(1, 0),
        // new Translation2d(1, -1)),
        // new Pose2d(2, -1, Rotation2d.fromDegrees(180)),
        // trajectoryConfig);

        // // 3. Define PID controllers for tracking trajectory
        // PIDController xController = new PIDController(AutoConstants.kPXController, 0,
        // 0);
        // PIDController yController = new PIDController(AutoConstants.kPYController, 0,
        // 0);
        // ProfiledPIDController thetaController = new ProfiledPIDController(
        // AutoConstants.kPThetaController, 0, 0,
        // AutoConstants.kThetaControllerConstraints);
        // thetaController.enableContinuousInput(-Math.PI, Math.PI);

        // // 4. Construct command to follow trajectory
        // SwerveControllerCommand swerveControllerCommand = new
        // SwerveControllerCommand(
        // trajectory,
        // swerveSubsystem::getPose,
        // DriveConstants.kDriveKinematics,
        // xController,
        // yController,
        // thetaController,
        // swerveSubsystem::setModuleStates,
        // swerveSubsystem);

        // // 5. Add some init and wrap-up, and return everything
        // return new SequentialCommandGroup(new InstantCommand(() ->
        // swerveSubsystem.resetOdometry(trajectory.getInitialPose())),
        // swerveControllerCommand, new InstantCommand(() ->
        // swerveSubsystem.stopModules()));
        //

    }

}
