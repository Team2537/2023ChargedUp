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
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.IOConstants;
import static frc.robot.Constants.ArmConstants.*;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;

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
  private final XboxController m_controller = new XboxController(IOConstants.kXboxControllerPort);
  private final LogitechJoystick m_gunnerJoystick = new LogitechJoystick(1);

  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem m_swerveSubsystem = new SwerveSubsystem();
  private final ArmPivotSubsystem m_armPivotSubsystem = new ArmPivotSubsystem();
  private final ArmTelescopeSubsystem m_armTelescopeSubsystem = new ArmTelescopeSubsystem();
  private final GripperSubsystem m_gripperSubsystem = new GripperSubsystem(0, 0, i -> {
  }, 10, 11);

  private final FixedAngleCommand m_bottomRowAngle = new FixedAngleCommand(m_armPivotSubsystem, BOTTOM_ROW_ANGLE);
  private final FixedExtensionCommand m_bottomRowExtension = new FixedExtensionCommand(m_armTelescopeSubsystem,
      BOTTOM_ROW_EXTENSION);
  private final SetPositionCommand m_bottomRowPosition = new SetPositionCommand(m_bottomRowAngle, m_bottomRowExtension);

  // Position for middle cone row when bumpers are against the community
  private final FixedAngleCommand m_middleRowAngle = new FixedAngleCommand(m_armPivotSubsystem, MIDDLE_ROW_ANGLE);
  private final FixedExtensionCommand m_middleRowExtension = new FixedExtensionCommand(m_armTelescopeSubsystem,
      MIDDLE_ROW_EXTENSION);
  private final SetPositionCommand m_middleRowPosition = new SetPositionCommand(m_middleRowAngle, m_middleRowExtension);

  // Position for top cone row when bumpers are against the community
  private final FixedAngleCommand m_topRowAngle = new FixedAngleCommand(m_armPivotSubsystem, TOP_ROW_ANGLE);
  private final FixedExtensionCommand m_topRowExtension = new FixedExtensionCommand(m_armTelescopeSubsystem,
      TOP_ROW_EXTENSION);
  private final SetPositionCommand m_topRowPosition = new SetPositionCommand(m_topRowAngle, m_topRowExtension);

  private final FixedExtensionCommand m_grabExtension = new FixedExtensionCommand(m_armTelescopeSubsystem, GRAB_EXTENSION);
  private final FixedAngleCommand m_grabAngle = new FixedAngleCommand(m_armPivotSubsystem, GRAB_ANGLE);
  private final SetPositionCommand m_grabPosition = new SetPositionCommand(m_grabAngle, m_grabExtension);

  private final FixedExtensionCommand test = new FixedExtensionCommand(m_armTelescopeSubsystem, 5);
  private final HomingCommand m_homingCommand = new HomingCommand(m_armPivotSubsystem, m_armTelescopeSubsystem);

  private final FixedAngleCommand testAngle = new FixedAngleCommand(m_armPivotSubsystem, 35);
  private final SetPositionCommand testPosition = new SetPositionCommand(testAngle, test);

  private final OpenGripperCommand openGripper = new OpenGripperCommand(m_gripperSubsystem);
  private final CloseGripperCommand closeGripper = new CloseGripperCommand(m_gripperSubsystem);
  private final ToggleGripperCommand toggleGripper = new ToggleGripperCommand(m_gripperSubsystem);

  private final FixedExtensionCommand m_returnExtension = new FixedExtensionCommand(m_armTelescopeSubsystem, 0);

  private final ManualArmControlCommand m_manualControl = new ManualArmControlCommand(
      m_armPivotSubsystem,
      m_armTelescopeSubsystem,
      () -> m_gunnerJoystick.getAxis(1),
      () -> 0.1 * m_gunnerJoystick.getHatSwitch());

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    m_swerveSubsystem.setDefaultCommand(new SwerveTeleopCommand(
        m_swerveSubsystem,
        () -> -m_controller.getLeftY(), // xSpdFunction is for forward direction
        () -> -m_controller.getLeftX(),
        () -> -m_controller.getRightX(),
        () -> !m_controller.getAButton()));
    // Configure the button bindings
    configureButtonBindings();
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
    m_gunnerJoystick.getButton(11).onTrue(m_homingCommand);
    m_gunnerJoystick.getButton(5).onTrue(openGripper);
    m_gunnerJoystick.getButton(3).onTrue(closeGripper);
    
    //m_gunnerJoystick.getButton(2).onTrue(m_bottomRowPosition);
    m_gunnerJoystick.getButton(7).onTrue(m_middleRowPosition);
    m_gunnerJoystick.getButton(9).onTrue(m_topRowPosition);
    m_gunnerJoystick.getButton(10).onTrue(m_grabPosition);

    m_gunnerJoystick.getButton(4).onTrue(m_returnExtension);

    m_gunnerJoystick.getThrottle().whileTrue(m_manualControl);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // 1. Create trajectory settings
    TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
        AutoConstants.kMaxSpeedMps,
        AutoConstants.kMaxAccelerationMetersPerSecondSquared)
        .setKinematics(DriveConstants.kDriveKinematics);

    // 2. Generate trajectory
    Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
        new Pose2d(0, 0, new Rotation2d(0)),
        List.of(
            new Translation2d(1, 0),
            new Translation2d(1, -1)),
        new Pose2d(2, -1, Rotation2d.fromDegrees(180)),
        trajectoryConfig);

    // 3. Define PID controllers for tracking trajectory
    PIDController xController = new PIDController(AutoConstants.kPXController, 0, 0);
    PIDController yController = new PIDController(AutoConstants.kPYController, 0, 0);
    ProfiledPIDController thetaController = new ProfiledPIDController(
        AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    // 4. Construct command to follow trajectory
    SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
        trajectory,
        m_swerveSubsystem::getPose,
        DriveConstants.kDriveKinematics,
        xController,
        yController,
        thetaController,
        m_swerveSubsystem::setModuleStates,
        m_swerveSubsystem);

    // 5. Add some init and wrap-up, and return everything
    return new SequentialCommandGroup(
        new InstantCommand(() -> m_swerveSubsystem.resetOdometry(trajectory.getInitialPose())), swerveControllerCommand,
        new InstantCommand(() -> m_swerveSubsystem.stopModules()));
  }

}
