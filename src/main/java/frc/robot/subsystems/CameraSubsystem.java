package frc.robot.subsystems;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CameraSubsystem extends SubsystemBase {

    public CameraSubsystem() {
        CameraServer.startAutomaticCapture();
    }

    @Override
    public void periodic() {}
}
