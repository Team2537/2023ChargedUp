package frc.robot;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

public class KeyboardControls {

    private final SendableChooser<Double> forward, left, rotate;
    private final SendableChooser<Boolean> claw;
    private final SendableChooser<Integer> position;

    public KeyboardControls() {
        ShuffleboardTab tab = Shuffleboard.getTab("Keyboard Controls");

        forward = new SendableChooser<>();
        forward.addOption("forward", 0.5);
        forward.addOption("backward", -0.5);
        forward.addOption("stop", 0.0);
        forward.setDefaultOption("stop", 0.0);

        tab.add(forward);

        left = new SendableChooser<>();
        left.addOption("right", -0.5);
        left.addOption("left", 0.5);
        left.addOption("stop", 0.0);
        left.setDefaultOption("stop", 0.0);

        tab.add(left);

        rotate = new SendableChooser<>();
        rotate.addOption("cw", -0.5);
        rotate.addOption("ccw", 0.5);
        rotate.addOption("stop", 0.0);
        rotate.setDefaultOption("stop", 0.0);

        tab.add(rotate);

        claw = new SendableChooser<>();
        claw.addOption("open", true);
        claw.addOption("close", false);
        claw.setDefaultOption("close", false);

        tab.add(claw);

        position = new SendableChooser<>();
        position.addOption("none", 0);
        position.addOption("home", 1);
        position.addOption("shelf", 2);
        position.setDefaultOption("none", 0);

        tab.add(position);
    }

    public double getForward() {
        return forward.getSelected();
    }

    public double getLeft() {
        return left.getSelected();
    }

    public double getRotate() {
        return rotate.getSelected();
    }

    public boolean getClaw() {
        return claw.getSelected();
    }

    public int getPosition() {
        return position.getSelected();
    }
}
