// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import javax.xml.namespace.QName;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.CloseGripperCommand;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.OpenGripperCommand;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.GripperSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  Joystick m_GunnerJoystick = new Joystick(0);

  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private final GripperSubsystem m_gripperSubsystem = new GripperSubsystem(0, 0, null, 10, 11);

  private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem);
  private final OpenGripperCommand openGripper = new OpenGripperCommand(m_gripperSubsystem);
  private final CloseGripperCommand closeGripper = new CloseGripperCommand(m_gripperSubsystem);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {                                                                                                                               
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    Trigger closeGrip = new Trigger(() -> m_GunnerJoystick.getRawButton(11));
    closeGrip.onTrue(closeGripper);

    Trigger openGrip = new Trigger(() -> m_GunnerJoystick.getRawButton(12));
    openGrip.onTrue(closeGripper);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return m_autoCommand;
  }
}
