package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.GameController;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;


public class RGBSubsystem extends SubsystemBase{

    private PWM lightLine;
    
    private GameController m_controller;

    private ShuffleboardTab pwmTab;

    private int targetValue;

    /*
     * Purple: 
     * Yellow: 
     */

    public RGBSubsystem(int port, int control){
        lightLine = new PWM(port);
        m_controller = new GameController(control);        

        pwmTab = Shuffleboard.getTab("Test");

        pwmTab.addInteger("PWM", () -> targetValue);
        pwmTab.addInteger("Raw PWM", () -> lightLine.getRaw());



    }

    public void setPurple(){

    }
    
    public void setYellow(){

    }

    @Override
    public void periodic(){
        targetValue = m_controller.getRawDPad() > -1 ? m_controller.getRawDPad() %255 : 0;
        lightLine.setRaw(m_controller.getRawDPad() > -1 ? m_controller.getRawDPad() %255 : 0);
        
    }


}
