// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.SetColorCommand;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.RGBSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import static frc.robot.constants.Constants.*;
import static frc.robot.constants.Ports.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...


  XboxController m_controller = new XboxController(0);

  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private final RGBSubsystem m_RgbSubsystem = new RGBSubsystem(0, 0);

  private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem);

  private final SetColorCommand redColor = new SetColorCommand(m_RgbSubsystem, RED);
  private final SetColorCommand yellowColor = new SetColorCommand(m_RgbSubsystem, YELLOW);
  private final SetColorCommand purpleColor = new SetColorCommand(m_RgbSubsystem, PURPLE);
  private final SetColorCommand greenColor = new SetColorCommand(m_RgbSubsystem, GREEN);
  private final SetColorCommand awesomeColor = new SetColorCommand(m_RgbSubsystem, AWESOME);

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
