package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * Wrapper class for {@link Joystick}
 * <p>Some additional functionality for ease of use:</p>
 * Converts POV to an axis, Returns buttons as {@link Trigger}
 */
public class LogitechJoystick {
    private final Joystick m_joystick;

    /**
     * @param port USB port ID, usually 0 (check in Driver Station)
     */
    public LogitechJoystick(int port) {
        m_joystick = new Joystick(port);
    }

    /**
     * Get the joystick hat switch as an axis
     * @return a double -1 to 1 representing the Y-axis of the hat switch
     */
    public double getHatSwitch() {
        return m_joystick.getPOV() == -1 ? 0 : Math.cos(Math.toRadians(m_joystick.getPOV()));
    }


    /**
     * Gets an axis from the joystick
     * @param axis the number of the axis
     * @return the axis reading from -1 to 1, (0 is default)
     */
    public double getAxis(int axis) {
        return m_joystick.getRawAxis(axis);
    }

    public Trigger getThrottle() {
        return new Trigger(() -> getAxis(3) < 0.5);
    }

    /**
     * Get a button from the joystick as a {@link Trigger}
     * @param button the raw ID of the button
     * @return a Trigger representing when the button is pressed
     */
    public Trigger getButton(int button) {
        return new Trigger(() -> m_joystick.getRawButton(button));
    }
}
