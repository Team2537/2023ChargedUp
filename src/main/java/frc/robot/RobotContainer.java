// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.IOConstants;
import frc.robot.Constants.LidarConstants;
import frc.robot.commandGroups.SetPositionCommandGroup;

import static frc.robot.Constants.ArmConstants.*;
import static frc.robot.Constants.ColorConstants.*;

import java.util.function.Supplier;

import frc.robot.commands.*;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandGroupBase;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

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
        public final XboxController m_controller = new XboxController(IOConstants.kXboxControllerPort); // TODO: make
                                                                                                        // private
        private final LogitechJoystick m_gunnerJoystick = new LogitechJoystick(1);

        // The robot's subsystems and commands are defined here...
        private final SwerveSubsystem m_swerveSubsystem = new SwerveSubsystem();
        private final ArmPivotSubsystem m_armPivotSubsystem = new ArmPivotSubsystem();
        private final ArmTelescopeSubsystem m_armTelescopeSubsystem = new ArmTelescopeSubsystem();
        private final GripperSubsystem m_gripperSubsystem = new GripperSubsystem(LidarConstants.kTargetLow,
                        LidarConstants.kTargetHigh, LidarConstants.kLidarReadPort, LidarConstants.kLidarTriggerPort);
        private final LockCommand lockCommand = new LockCommand(m_swerveSubsystem);
        private final ZeroHeadingCommand m_zeroHeadingCommand = new ZeroHeadingCommand(m_swerveSubsystem);
        private final CameraSubsystem m_cameraSubsystem = new CameraSubsystem();

        private final RGBSubsystem m_RgbSubsystem = new RGBSubsystem(1);

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
        private final SetPositionCommandGroup m_topRowPosition = new SetPositionCommandGroup(m_topRowAngle,
                        m_topRowExtension);

        // positions for picking up from ground
        private final FixedExtensionCommand m_grabExtension = new FixedExtensionCommand(m_armTelescopeSubsystem,
                        GRAB_EXTENSION);
        private final FixedAngleCommand m_grabAngle = new FixedAngleCommand(m_armPivotSubsystem, GRAB_ANGLE);
        private final SetPositionCommandGroup m_grabPosition = new SetPositionCommandGroup(m_grabAngle,
                        m_grabExtension);

        private final FixedExtensionCommand m_shelfExtension = new FixedExtensionCommand(m_armTelescopeSubsystem,
                        SHELF_EXTENSION);
        private final FixedAngleCommand m_shelfAngle = new FixedAngleCommand(m_armPivotSubsystem, SHELF_ANGLE);
        private final SetPositionCommandGroup m_shelfPosition = new SetPositionCommandGroup(m_shelfAngle,
                        m_shelfExtension);

        private final FixedExtensionCommand test = new FixedExtensionCommand(m_armTelescopeSubsystem, 5);
        private final HomingCommand m_homingCommand = new HomingCommand(m_armPivotSubsystem, m_armTelescopeSubsystem);

        private final ArmEncoderSyncCommand m_encoderSync = new ArmEncoderSyncCommand(m_armPivotSubsystem);

        private final ToggleGripperCommand toggleGripper = new ToggleGripperCommand(m_gripperSubsystem);

        private final AutoGrabCommand m_autoGrabCommand = new AutoGrabCommand(m_gripperSubsystem);

        private final FixedExtensionCommand m_returnExtension = new FixedExtensionCommand(m_armTelescopeSubsystem, 0);

        private final ManualArmControlCommand m_manualControl = new ManualArmControlCommand(
                        m_armPivotSubsystem,
                        m_armTelescopeSubsystem,
                        () -> m_gunnerJoystick.getAxis(1),
                        () -> 0.1 * m_gunnerJoystick.getHatSwitch());


        private final SetColorCommand redColor = new SetColorCommand(m_RgbSubsystem, RED);
        private final SetColorCommand yellowColor = new SetColorCommand(m_RgbSubsystem, YELLOW);
        private final SetColorCommand purpleColor = new SetColorCommand(m_RgbSubsystem, PURPLE);
        private final SetColorCommand greenColor = new SetColorCommand(m_RgbSubsystem, GREEN);
        private final SetColorCommand awesomeColor = new SetColorCommand(m_RgbSubsystem, AWESOME);

        private final SendableChooser<Supplier<Command>> m_autoChooser = new SendableChooser<>();

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
                                () -> m_controller.getLeftTriggerAxis()> 0.75));

                // Configure the button bindings
                configureButtonBindings();
                m_autoChooser.addOption("Test Drive Cmd", () -> testDriveCmd());
                m_autoChooser.addOption("Red Two Piece Auto", () -> redGetTwoPieceCmd());
                m_autoChooser.addOption("Blue Two Piece Auto", () -> blueGetTwoPieceCmd());
                m_autoChooser.addOption("One Gamepiece and Back", () -> placeAndDriveCommand());
                m_autoChooser.addOption("One Gamepiece", () -> getPlaceCmd());
                // m_autoChooser.addOption("Super Extension", () -> superExtensionCommand());
                
                m_autoChooser.addOption("Test Auto Grab", () -> testAutoGrab());
                m_autoChooser.addOption("Test Auto Balance", () -> testAutoBalance());
               
                Shuffleboard.getTab("Autonomous").add(m_autoChooser);

                Shuffleboard.getTab("controls").addNumber("POV", () -> m_controller.getPOV());

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

                yButton.toggleOnTrue(lockCommand);

                Trigger bothTriggers = new Trigger(
                                () -> (m_controller.getLeftTriggerAxis() > 0.75
                                                && m_controller.getRightTriggerAxis() > 0.75));

                bothTriggers.onTrue(m_zeroHeadingCommand);

                m_gunnerJoystick.getButton(8).onTrue(m_homingCommand);
                m_gunnerJoystick.getButton(2).onTrue(toggleGripper);

                m_gunnerJoystick.getButton(12).onTrue(m_encoderSync);

                m_gunnerJoystick.getButton(9).onTrue(m_middleRowPosition);
                m_gunnerJoystick.getButton(7).onTrue(m_topRowPosition);
                m_gunnerJoystick.getButton(4).onTrue(m_grabPosition);
                m_gunnerJoystick.getButton(10).onTrue(m_shelfPosition);

                m_gunnerJoystick.getButton(6).onTrue(m_returnExtension);
                m_gunnerJoystick.getThrottle().whileTrue(m_manualControl);

                m_gunnerJoystick.getButton(5).toggleOnTrue(purpleColor);
                m_gunnerJoystick.getButton(3).toggleOnTrue(yellowColor);

                m_gunnerJoystick.getButton(1).toggleOnTrue(m_autoGrabCommand);

                // Trigger moveForwardTrigger = new Trigger(() -> m_controller.getPOV() == 0);
                Trigger moveRightTrigger = new Trigger(() -> m_controller.getPOV() > 0 && m_controller.getPOV() < 180);
                // Trigger moveBackTrigger = new Trigger(() -> m_controller.getPOV() == 180);
                Trigger moveLeftTrigger = new Trigger(() -> m_controller.getPOV() > 180 && m_controller.getPOV() < 360);

                // moveForwardTrigger.whileTrue(new SetChassisStateCommand(m_swerveSubsystem, () -> 0.1, () -> 0.0, null));
                moveRightTrigger.whileTrue(new SetChassisStateCommand(m_swerveSubsystem, () -> 0.0, () -> -0.1, null));
                // moveBackTrigger.whileTrue(new SetChassisStateCommand(m_swerveSubsystem, () -> -0.1, () -> 0.0, null));
                moveLeftTrigger.whileTrue(new SetChassisStateCommand(m_swerveSubsystem, () -> 0.0, () -> 0.1, null));
        }

        public Command superExtensionCommand() {
                return  new FixedAngleCommand(m_armPivotSubsystem, 15.66).andThen(
                        new FixedExtensionCommand(m_armTelescopeSubsystem, 8.2)).andThen(
                        new OpenGripperCommand(m_gripperSubsystem)).andThen(
                        new WaitCommand(0.3)).andThen(
                        new FixedAngleCommand(m_armPivotSubsystem, -22).andThen(
                        new FixedExtensionCommand(m_armTelescopeSubsystem, 12.998)
                        ));
        }

        public Command testDriveCmd() {
                PathPlannerTrajectory trajPath = PathPlanner.loadPath("Red Path",
                new PathConstraints(1.0,
                       1.5));

                m_swerveSubsystem.resetOdometry(trajPath.getInitialState().poseMeters);
                m_swerveSubsystem.setHeading(trajPath.getInitialState().holonomicRotation.getDegrees());
       
             
                return new PathCommand(m_swerveSubsystem, trajPath);
        }

        public Command testAutoGrab() {
                return new SequentialCommandGroup(
                new HomingCommand(m_armPivotSubsystem, m_armTelescopeSubsystem),
                new FixedAngleCommand(m_armPivotSubsystem, GRAB_ANGLE),
                new FixedExtensionCommand(m_armTelescopeSubsystem, GRAB_EXTENSION),
                new OpenGripperCommand(m_gripperSubsystem),
                new AutonomousAutoGrabCommand(m_swerveSubsystem, m_gripperSubsystem)
                );
        }

        public Command testAutoBalance() {
                return new SequentialCommandGroup(
                new BalanceBackUpCommand(m_swerveSubsystem, 1.0),
                // new BalanceRampCommand(m_swerveSubsystem, 0.7, 1.25),
                new BalanceCommand(m_swerveSubsystem)
                );
        }

        public Command placeAndDriveCommand() {
                PathPlannerTrajectory trajPath = PathPlanner.loadPath("Backup",
                new PathConstraints(3.0,
                       3.5)); //meters per second //seems to work better when constraint acceleration is higher than path planner acceleration

                m_swerveSubsystem.resetOdometry(trajPath.getInitialState().poseMeters);
                m_swerveSubsystem.setHeading(trajPath.getInitialState().holonomicRotation.getDegrees());

                return new SequentialCommandGroup(
                        new HomingCommand(m_armPivotSubsystem, m_armTelescopeSubsystem),
                        new FixedAngleCommand(m_armPivotSubsystem, 26.9), //15.66
                        new FixedExtensionCommand(m_armTelescopeSubsystem, 11.0), //8.2
                        new OpenGripperCommand(m_gripperSubsystem),
                        new WaitCommand(0.1),
                        new ParallelCommandGroup(
                                new SequentialCommandGroup(
                                        //new HomingCommand(m_armPivotSubsystem, m_armTelescopeSubsystem),
                                        new FixedExtensionCommand(m_armTelescopeSubsystem, GRAB_EXTENSION),
                                        new FixedAngleCommand(m_armPivotSubsystem, GRAB_ANGLE)
                                ),
                                new PathCommand(m_swerveSubsystem, trajPath)
                                
                        )
                );
                               
        }

        public Command blueGetTwoPieceCmd() {
                PathPlannerTrajectory trajPath = PathPlanner.loadPath("Path",
                new PathConstraints(3.0,
                       3.5)); //meters per second //seems to work better when constraint acceleration is higher than path planner acceleration
                       PathPlannerTrajectory trajReversePath = PathPlanner.loadPath("Reverse Path",
                       new PathConstraints(3.0,
                              3.5));
                // PathPlannerTrajectory.transformTrajectoryForAlliance(trajPath, DriverStation.Alliance.Red);
                // PathPlannerTrajectory.transformTrajectoryForAlliance(trajReversePath,
                // DriverStation.Alliance.Red);

                m_swerveSubsystem.resetOdometry(trajPath.getInitialState().poseMeters);
                m_swerveSubsystem.setHeading(trajPath.getInitialState().holonomicRotation.getDegrees());

                return new SequentialCommandGroup(
                        new FixedAngleCommand(m_armPivotSubsystem, 26.9), //15.66
                        new FixedExtensionCommand(m_armTelescopeSubsystem, 11.0), //8.2
                        new OpenGripperCommand(m_gripperSubsystem),
                        new WaitCommand(0.1),
                        new ParallelCommandGroup(
                                new SequentialCommandGroup(
                                        //new HomingCommand(m_armPivotSubsystem, m_armTelescopeSubsystem),
                                        new FixedExtensionCommand(m_armTelescopeSubsystem, GRAB_EXTENSION),
                                        new FixedAngleCommand(m_armPivotSubsystem, GRAB_ANGLE)
                                ),
                                new PathCommand(m_swerveSubsystem, trajPath)
                                
                        ),
                        new AutonomousAutoGrabCommand(m_swerveSubsystem, m_gripperSubsystem),
                        new ParallelCommandGroup(
                                new SequentialCommandGroup(
                                        new WaitUntilDesiredHeadingCommand(m_swerveSubsystem, Rotation2d.fromDegrees(180.0)),
                                        new FixedAngleCommand(m_armPivotSubsystem, 15.66),
                                        new FixedExtensionCommand(m_armTelescopeSubsystem, 8.2)     
                                ),    
                                new PathCommand(m_swerveSubsystem, trajReversePath)
                                
                        )//,
                        //new OpenGripperCommand(m_gripperSubsystem)
                        
                );
        }

        public Command redGetTwoPieceCmd() {
                PathPlannerTrajectory trajPath = PathPlanner.loadPath("Red Path",
                new PathConstraints(3.0,
                       3.5)); //meters per second //seems to work better when constraint acceleration is higher than path planner acceleration
                       PathPlannerTrajectory trajReversePath = PathPlanner.loadPath("Red Reverse Path",
                       new PathConstraints(3.0,
                              3.5));

                m_swerveSubsystem.resetOdometry(trajPath.getInitialState().poseMeters);
                m_swerveSubsystem.setHeading(trajPath.getInitialState().holonomicRotation.getDegrees());

                return new SequentialCommandGroup(
                        new FixedAngleCommand(m_armPivotSubsystem, 26.9), //15.66
                        new FixedExtensionCommand(m_armTelescopeSubsystem, 11.0), //8.2
                        new OpenGripperCommand(m_gripperSubsystem),
                        new WaitCommand(0.1),
                        new ParallelCommandGroup(
                                new SequentialCommandGroup(
                                        //new HomingCommand(m_armPivotSubsystem, m_armTelescopeSubsystem),
                                        new FixedExtensionCommand(m_armTelescopeSubsystem, GRAB_EXTENSION),
                                        new FixedAngleCommand(m_armPivotSubsystem, GRAB_ANGLE)
                                ),
                                new PathCommand(m_swerveSubsystem, trajPath)
                                
                        ),
                        new AutonomousAutoGrabCommand(m_swerveSubsystem, m_gripperSubsystem),
                        new ParallelDeadlineGroup(
                                new PathCommand(m_swerveSubsystem, trajReversePath), //ends parallel deadline group when path command ends
                                new SequentialCommandGroup(
                                        new WaitUntilDesiredHeadingCommand(m_swerveSubsystem, Rotation2d.fromDegrees(180.0)),
                                        new FixedAngleCommand(m_armPivotSubsystem, 15.66),
                                        new FixedExtensionCommand(m_armTelescopeSubsystem, 8.2)     
                                )     
                        ),
                        new OpenGripperCommand(m_gripperSubsystem)
                        
                );
        }

      // we did this on Thursday instead of parallel deadline group
        // new ParallelCommandGroup(
        //         new SequentialCommandGroup(
        //                 new WaitUntilDesiredHeadingCommand(m_swerveSubsystem, Rotation2d.fromDegrees(180.0)),
        //                 new FixedAngleCommand(m_armPivotSubsystem, 15.66),
        //                 new FixedExtensionCommand(m_armTelescopeSubsystem, 8.2)     
        //         ),
        //         new PathCommand(m_swerveSubsystem, trajReversePath)
        // ),
       

        public Command getPlaceCmd() {
              
                return new HomingCommand(m_armPivotSubsystem,
                m_armTelescopeSubsystem).andThen(
                        new FixedAngleCommand(m_armPivotSubsystem, 15.66)).andThen(
                        new FixedExtensionCommand(m_armTelescopeSubsystem, 8.2)).andThen(
                        new OpenGripperCommand(m_gripperSubsystem)).andThen(
                        new WaitCommand(0.5)).andThen(
                        new HomingCommand(m_armPivotSubsystem, m_armTelescopeSubsystem)
                );
        }
        /**
         * Use this to pass the autonomous command to the main {@link Robot} class.
         *
         * @return the command to run in autonomous
         */
        public Command getAutonomousCommand() {

                // one piece path
                // PathPlannerTrajectory trajPath = PathPlanner.loadPath("Path",
                //         new PathConstraints(AutoConstants.kMaxSpeedMps,
                //                 AutoConstants.kMaxAccelerationMetersPerSecondSquared));

                // two piece path
                // PathPlannerTrajectory trajPath = PathPlanner.loadPath("Path",
                // new PathConstraints(AutoConstants.kMaxSpeedMps,
                // AutoConstants.kMaxAccelerationMetersPerSecondSquared));
                // PathPlannerTrajectory trajReversePath = PathPlanner.loadPath("Reverse Path",
                // new PathConstraints(AutoConstants.kMaxSpeedMps,
                // AutoConstants.kMaxAccelerationMetersPerSecondSquared));

                // center path
                // PathPlannerTrajectory trajPath = PathPlanner.loadPath("CenterBalancePt1",
                // new PathConstraints(1.0, 2.0));
                // PathPlannerTrajectory trajReversePath =
                // PathPlanner.loadPath("CenterBalancePt2",
                // new PathConstraints(0.5, 2.0));

                // PathPlannerTrajectory.transformTrajectoryForAlliance(trajPath, DriverStation.getAlliance());
                // PathPlannerTrajectory.transformTrajectoryForAlliance(trajReversePath,
                // DriverStation.getAlliance());

                // m_swerveSubsystem.resetOdometry(trajPath.getInitialState().poseMeters);
                // m_swerveSubsystem.setHeading(trajPath.getInitialState().holonomicRotation.getDegrees());

                // center - cube and balance
                // return new HomingCommand(m_armPivotSubsystem,
                // m_armTelescopeSubsystem).andThen(
                // new FixedAngleCommand(m_armPivotSubsystem, 15.66)).andThen(
                // new FixedExtensionCommand(m_armTelescopeSubsystem, 8.2)).andThen(
                // new OpenGripperCommand(m_gripperSubsystem)).andThen(
                // new WaitCommand(0.5)).andThen(
                // new HomingCommand(m_armPivotSubsystem, m_armTelescopeSubsystem).alongWith(
                // new PathCommand(m_swerveSubsystem, trajPath)).andThen(
                // new BalanceCommand(m_swerveSubsystem).andThen(
                // new LockCommand(m_swerveSubsystem)
                // )));

                // return new SwerveHomingCommand(m_swerveSubsystem).andThen(new
                // PathCommand(m_swerveSubsystem, trajectory).andThen(
                // new BalanceCommand(m_swerveSubsystem).andThen(new
                // LockCommand(m_swerveSubsystem))));

                // return new SwerveHomingCommand(m_swerveSubsystem).andThen(
                // new PathCommand(m_swerveSubsystem, trajPath).andThen(
                // new BalanceCommand(m_swerveSubsystem)).andThen(
                // new LockCommand(m_swerveSubsystem)));

                // one cube
                // return new HomingCommand(m_armPivotSubsystem,
                // m_armTelescopeSubsystem).andThen(
                // new FixedAngleCommand(m_armPivotSubsystem, 15.66)).andThen(
                // new FixedExtensionCommand(m_armTelescopeSubsystem, 8.2)).andThen(
                // new OpenGripperCommand(m_gripperSubsystem)).andThen(
                // new WaitCommand(0.5)).andThen(
                // new HomingCommand(m_armPivotSubsystem, m_armTelescopeSubsystem).andThen(
                // new PathCommand(m_swerveSubsystem, trajPath).andThen(
                // new RotateCommand(m_swerveSubsystem, Rotation2d.fromDegrees(180.0)
                // ))));

                return m_autoChooser.getSelected().get();
                // two cubes
                // return new HomingCommand(m_armPivotSubsystem,
                // m_armTelescopeSubsystem).andThen(
                // new FixedAngleCommand(m_armPivotSubsystem, 15.66)).andThen(
                // new FixedExtensionCommand(m_armTelescopeSubsystem, 8.2)).andThen(
                // new OpenGripperCommand(m_gripperSubsystem)).andThen(
                // new WaitCommand(0.5)).andThen(
                // new HomingCommand(m_armPivotSubsystem, m_armTelescopeSubsystem).andThen(
                // new PathCommand(m_swerveSubsystem, trajPath)).andThen(
                // new RotateCommand(m_swerveSubsystem, Rotation2d.fromDegrees(180.0)). andThen(
                // new FixedAngleCommand(m_armPivotSubsystem, GRAB_ANGLE).andThen(
                // new FixedExtensionCommand(m_armTelescopeSubsystem, GRAB_EXTENSION).andThen(
                // new AutonomousAutoGrabCommand(m_swerveSubsystem, m_gripperSubsystem).andThen(
                // new HomingCommand(m_armPivotSubsystem, m_armTelescopeSubsystem)).andThen(
                // new RotateCommand(m_swerveSubsystem, Rotation2d.fromDegrees(180.0)).andThen(
                // new PathCommand(m_swerveSubsystem, trajReversePath).andThen(
                // new FixedAngleCommand(m_armPivotSubsystem, 15.66)).andThen(
                // new FixedExtensionCommand(m_armTelescopeSubsystem, 4.2)).andThen(
                // new OpenGripperCommand(m_gripperSubsystem)).andThen(
                // new WaitCommand(0.5)).andThen(
                // new HomingCommand(m_armPivotSubsystem, m_armTelescopeSubsystem
                // )))))))));

                // cone
                // return new HomingCommand(m_armPivotSubsystem,
                // m_armTelescopeSubsystem).andThen(
                // new FixedAngleCommand(m_armPivotSubsystem, 31.07)).andThen(
                // new FixedExtensionCommand(m_armTelescopeSubsystem, 13.17)).andThen(
                // new OpenGripperCommand(m_gripperSubsystem)).andThen(
                // new WaitCommand(0.5)).andThen(
                // new HomingCommand(m_armPivotSubsystem, m_armTelescopeSubsystem)/* .alongWith(
                // new PathCommand(m_swerveSubsystem, trajectory)).andThen(
                // new BalanceCommand(m_swerveSubsystem)).andThen(
                // new LockCommand(m_swerveSubsystem))*/) ;

                // return new HomingCommand(m_armPivotSubsystem,
                // m_armTelescopeSubsystem).andThen(
                // new FixedAngleCommand(m_armPivotSubsystem, 15.66)).andThen(
                // new FixedExtensionCommand(m_armTelescopeSubsystem, 8.2)).andThen(
                // new OpenGripperCommand(m_gripperSubsystem)).andThen(
                // new WaitCommand(0.5)).andThen(
                // new HomingCommand(m_armPivotSubsystem, m_armTelescopeSubsystem).andThen(
                // new PathCommand(m_swerveSubsystem, trajPath)));

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
