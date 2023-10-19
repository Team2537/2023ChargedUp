// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.Auton;
import frc.robot.Constants.IOConstants;
import frc.robot.Constants.LidarConstants;
import frc.robot.commandGroups.SetPositionCommandGroup;

import static frc.robot.Constants.ArmConstants.*;
import static frc.robot.Constants.ColorConstants.*;

import java.io.File;
import java.util.HashMap;
import java.util.List;
import java.util.function.Supplier;

import javax.swing.plaf.TreeUI;
import javax.xml.crypto.dsig.spec.HMACParameterSpec;

import frc.robot.commands.*;
import frc.robot.commands.swerve.AbsoluteDrive;
import frc.robot.commands.swerve.FollowTrajectory;
import frc.robot.commands.swerve.TeleopDrive;
import frc.robot.commands.swerve.WaitUntilHeading;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandGroupBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

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
        public final CommandXboxController m_controller = new CommandXboxController(IOConstants.kXboxControllerPort); // TODO: make
                                                                                                        // private
        private final LogitechJoystick m_gunnerJoystick = new LogitechJoystick(1);

        // The robot's subsystems and commands are defined here...
        private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve"));
        private final ArmPivotSubsystem m_armPivotSubsystem = new ArmPivotSubsystem();
        private final ArmTelescopeSubsystem m_armTelescopeSubsystem = new ArmTelescopeSubsystem();
        private final GripperSubsystem m_gripperSubsystem = new GripperSubsystem(LidarConstants.kTargetLow,
                        LidarConstants.kTargetHigh, LidarConstants.kLidarReadPort, LidarConstants.kLidarTriggerPort);
        private final CameraSubsystem m_cameraSubsystem = new CameraSubsystem();

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

        private final SendableChooser<Supplier<Command>> m_autoChooser = new SendableChooser<>();
        private HashMap<String, Command> eventMap = new HashMap<>();

        /**
         * The container for the robot. Contains subsystems, OI devices, and commands.
         */
        public RobotContainer() {

                TeleopDrive closedTeleopDrive = new TeleopDrive(drivebase,
                () -> MathUtil.applyDeadband(m_controller.getLeftY(), Constants.OperatorConstants.LEFT_Y_DEADBAND),
                () -> MathUtil.applyDeadband(m_controller.getLeftX(), Constants.OperatorConstants.LEFT_X_DEADBAND),
                () -> -m_controller.getRightX(),
                () -> !m_controller.getHID().getLeftBumper(),
                false,
                false);

                // Configure the button bindings
                configureButtonBindings();
               
                Shuffleboard.getTab("Autonomous").add(m_autoChooser);

                m_autoChooser.addOption("Blue Two Piece", () -> blueTwoPiece());
                m_autoChooser.addOption("Straight Line", () -> testDrive());
                m_autoChooser.addOption("Test Home and Place", () -> testHomeDrive());
                m_autoChooser.addOption("Place and Drive", () -> placeAndDrive());
                m_autoChooser.addOption("Test Event", () -> testEvent());

                drivebase.setDefaultCommand(closedTeleopDrive);

                // Add eventmap markers
                eventMap.put("event", new PrintCommand("we did it"));
                eventMap.put("home", new HomingCommand(m_armPivotSubsystem, m_armTelescopeSubsystem));
                eventMap.put("angleMid", new FixedAngleCommand(m_armPivotSubsystem, Constants.ArmConstants.MIDDLE_ROW_ANGLE));
                eventMap.put("extendMid", new FixedExtensionCommand(m_armTelescopeSubsystem, Constants.ArmConstants.MIDDLE_ROW_EXTENSION));
                eventMap.put("openGripper", new OpenGripperCommand(m_gripperSubsystem));
                
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
                // Trigger rightTrigger = new Trigger(() -> controller.getRightTriggerAxis() >
                // 0.9);
                // rightTrigger.onTrue(new LockCommand(swerveSubsystem));



                m_controller.y().toggleOnTrue(new InstantCommand(drivebase::lock));
                m_controller.a().onTrue(new InstantCommand(drivebase::zeroGyro));

                m_controller.x().onTrue(new InstantCommand(() -> drivebase.postTrajectory(Constants.Auto.straightLine)));

                m_gunnerJoystick.getButton(8).onTrue(m_homingCommand);
                m_gunnerJoystick.getButton(2).onTrue(toggleGripper);

                // m_gunnerJoystick.getButton(2).onTrue(m_bottomRowPosition);
                m_gunnerJoystick.getButton(12).onTrue(m_encoderSync);

                // m_gunnerJoystick.getButton(11).onTrue(m_bottomRowPosition);
                m_gunnerJoystick.getButton(9).onTrue(m_middleRowPosition);
                m_gunnerJoystick.getButton(7).onTrue(m_topRowPosition);
                m_gunnerJoystick.getButton(4).onTrue(m_grabPosition);
                m_gunnerJoystick.getButton(10).onTrue(m_shelfPosition);

                m_gunnerJoystick.getButton(6).onTrue(m_returnExtension);
                m_gunnerJoystick.getThrottle().whileTrue(m_manualControl);

                m_gunnerJoystick.getButton(5).toggleOnTrue(purpleColor);
                m_gunnerJoystick.getButton(3).toggleOnTrue(yellowColor);

                m_gunnerJoystick.getButton(1).toggleOnTrue(m_autoGrabCommand);


        }



        
        public Command placeAndDrive(){
                return new SequentialCommandGroup(
                new HomingCommand(m_armPivotSubsystem, m_armTelescopeSubsystem),
                new SetPositionCommandGroup(m_middleRowAngle, m_middleRowExtension),
                new OpenGripperCommand(m_gripperSubsystem),
                new WaitCommand(0.1),
                new HomingCommand(m_armPivotSubsystem, m_armTelescopeSubsystem),
                new ParallelCommandGroup(
                        new SetPositionCommandGroup(m_grabAngle, m_grabExtension),
                        Commands.sequence(new FollowTrajectory(drivebase, Constants.Auto.backupTraj, true))
                
                ));
        }

        public Command blueTwoPiece(){
                return new SequentialCommandGroup(
                        new SetPositionCommandGroup(m_topRowAngle, m_topRowExtension),
                        new OpenGripperCommand(m_gripperSubsystem),
                        new WaitCommand(0.1),
                        new ParallelCommandGroup(                           
                                new SetPositionCommandGroup(m_grabAngle, m_grabExtension),
                                new FollowTrajectory(drivebase, Constants.Auto.blueTwoPieceTraj, true)
                        ),
                        new AutonomousAutoGrabCommand(drivebase, m_gripperSubsystem),
                        new ParallelCommandGroup(
                                new SequentialCommandGroup(
                                        new WaitUntilHeading(drivebase, 180),
                                        new SetPositionCommandGroup(m_topRowAngle, m_topRowExtension)

                                ),
                                new FollowTrajectory(drivebase, Constants.Auto.blueTwoPieceTrajReverse, true)
                        ),
                        new OpenGripperCommand(m_gripperSubsystem)
                );
        }

        public Command redTwoPiece(){
                return new SequentialCommandGroup(
                        new SetPositionCommandGroup(m_topRowAngle, m_topRowExtension),
                        new OpenGripperCommand(m_gripperSubsystem),
                        new WaitCommand(0.1),
                        new ParallelCommandGroup(                           
                                new SetPositionCommandGroup(m_grabAngle, m_grabExtension),
                                new FollowTrajectory(drivebase, Constants.Auto.redTwoPieceTraj, true)
                        ),
                        new AutonomousAutoGrabCommand(drivebase, m_gripperSubsystem),
                        new ParallelCommandGroup(
                                new SequentialCommandGroup(
                                        new WaitUntilHeading(drivebase, 180),
                                        new SetPositionCommandGroup(m_topRowAngle, m_topRowExtension)

                                ),
                                new FollowTrajectory(drivebase, Constants.Auto.redTwoPieceTrajReverse, true)
                        ),
                        new OpenGripperCommand(m_gripperSubsystem)
                );
        }

        public Command basicPlace(){
                return new SequentialCommandGroup(
                        new HomingCommand(m_armPivotSubsystem, m_armTelescopeSubsystem),
                        new SetPositionCommandGroup(m_topRowAngle, m_topRowExtension),
                        new OpenGripperCommand(m_gripperSubsystem),
                        new WaitCommand(0.1),
                        new HomingCommand(m_armPivotSubsystem, m_armTelescopeSubsystem)
                );
        }


        public Command testDrive(){
                return new FollowTrajectory(drivebase, Constants.Auto.straightLine, true);
        }

        public CommandBase testEvent(){
                List<PathPlannerTrajectory> path = PathPlanner.loadPathGroup("blueTwoPieceExit", Constants.Auto.constraints);

                SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(drivebase::getPose, drivebase::resetOdometry, Constants.Auton.translationConstants, Constants.Auton.rotationConstants, drivebase::setChassisSpeeds, eventMap, drivebase);

                return Commands.sequence(autoBuilder.fullAuto(path));
        }

        public CommandBase testHomeDrive(){
                return new SequentialCommandGroup(
                        new HomingCommand(m_armPivotSubsystem, m_armTelescopeSubsystem),
                        Commands.sequence(new FollowTrajectory(drivebase, Constants.Auto.testDrive, true))
                );
        }

        public Command getAutonomousCommand(){
                return m_autoChooser.getSelected().get();
        }

                


        

}
