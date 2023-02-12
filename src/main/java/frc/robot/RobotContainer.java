// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.FixedAngleCommand;
import frc.robot.commands.FixedExtensionCommand;
import frc.robot.commands.HomingCommand;
import frc.robot.commands.SetPositionCommand;
import frc.robot.subsystems.ArmPivotSubsystem;
import frc.robot.subsystems.ArmTelescopeSubsystem;
import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import static frc.robot.constants.Constants.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  LogitechJoystick m_gunnerJoystick = new LogitechJoystick(0);

// The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private final ArmTelescopeSubsystem m_armTelescopeSubsystem = new ArmTelescopeSubsystem();
  private final ArmPivotSubsystem m_armPivotSubsystem = new ArmPivotSubsystem();

  private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem);
// Position for bottom row when bumpers are against the community
  private final FixedAngleCommand m_bottomRowAngle = new FixedAngleCommand(m_armPivotSubsystem, BOTTOM_ROW_ANGLE);
  private final FixedExtensionCommand m_bottomRowExtension = new FixedExtensionCommand(m_armTelescopeSubsystem, BOTTOM_ROW_EXTENSION);
  private final SetPositionCommand m_bottomRowPosition = new SetPositionCommand(m_bottomRowAngle, m_bottomRowExtension);
// Position for middle cone row when bumpers are against the community
  private final FixedAngleCommand m_middleRowAngle = new FixedAngleCommand(m_armPivotSubsystem, MIDDLE_ROW_ANGLE);
  private final FixedExtensionCommand m_middleRowExtension = new FixedExtensionCommand(m_armTelescopeSubsystem, MIDDLE_ROW_EXTENSION);
  private final SetPositionCommand m_middleRowPosition = new SetPositionCommand(m_middleRowAngle, m_middleRowExtension);
// Position for top cone row when bumpers are against the community
  private final FixedAngleCommand m_topRowAngle = new FixedAngleCommand(m_armPivotSubsystem, TOP_ROW_ANGLE);
  private final FixedExtensionCommand m_topRowExtension = new FixedExtensionCommand(m_armTelescopeSubsystem, TOP_ROW_EXTENSION);
  private final SetPositionCommand m_topRowPosition = new SetPositionCommand(m_topRowAngle, m_topRowExtension);

  private final FixedExtensionCommand test = new FixedExtensionCommand(m_armTelescopeSubsystem, 7);
  private final HomingCommand m_homingCommand = new HomingCommand(m_armPivotSubsystem, m_armTelescopeSubsystem);
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
  }


  /*
   * Method to bind buttons to predefined commands (Declared above)
   * Create a trigger object bound to a button, and then bind the trigger object to a command
   * Add the trigger conditions
   */
  private void configureButtonBindings() {
    Trigger bottomRowPositionButton = m_gunnerJoystick.getButton(8);
    bottomRowPositionButton.onTrue(test);

    Trigger homingButton = m_gunnerJoystick.getButton(9);
    homingButton.onTrue(m_homingCommand);
  }


  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return m_autoCommand;
  }
}
