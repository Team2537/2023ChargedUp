package frc.robot;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj.XboxController;

public class GameController {
    private final XboxController m_controller;

    public GameController(int port){
        m_controller = new XboxController(port);
    }

    public Trigger getButton(int button) {
        return new Trigger(() -> m_controller.getRawButton(button));
    }

    public double getAxis(int axis) {
        return m_controller.getRawAxis(axis);
    }

    public double getHatSwitch() {
        return m_controller.getPOV() == -1 ? 0 : Math.cos(Math.toRadians(m_controller.getPOV()));
    }

    
}
