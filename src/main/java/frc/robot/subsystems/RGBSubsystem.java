package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;


public class RGBSubsystem extends SubsystemBase{

    private PWM lightLine;
    
    private ShuffleboardTab pwmTab;

    private int targetValue;

    private boolean updatePause = false;

    public RGBSubsystem(int port, int control){
        lightLine = new PWM(port);

        pwmTab = Shuffleboard.getTab("RGB");

        pwmTab.addInteger("Target PWM", () -> targetValue);
    }

    /*
     * NGUYEN-EDELEN Protocol 2023:
     * 1: Purple -> 20
     * 2: Yellow -> 40
     * 3: Green -> 60
     * 4: Red -> 80
     * 7: Awesome -> 140
     */
    public void setCommand(int command) {
        // command should be 1-5

        // convert command to targetValue
        targetValue = command*20;

        // make sure to not send any commands for one pulse to avoid decoding errors on the arduino
        updatePause = true;
    }

    @Override
    public void periodic(){
        if (!updatePause) {
            lightLine.setRaw(targetValue);
        } else {
            updatePause = true;
        }
    }


}
