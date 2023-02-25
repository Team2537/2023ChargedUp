package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.wpilibj.DigitalOutput;


public class RGBSubsystem extends SubsystemBase{

    private DigitalOutput lightLine;

    /*
     * Green: 
     * Red: 
     * Purple: 
     * Yellow: 
     */

    public RGBSubsystem(int port){
        lightLine = new DigitalOutput(port);

        lightLine.enablePWM(0);
        
    }


}
