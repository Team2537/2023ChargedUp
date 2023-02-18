package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;

public class VisionSubsystem extends SubsystemBase{

// Defines the logs
    private static DoubleLogEntry XOffsetLog;
    private static DoubleLogEntry YOffsetLog;
    private static DoubleLogEntry AreaLog;
    private static DoubleLogEntry SkewLog;

//Defines the sensor readings
    private double x;
    private double y;
    private double area;
    private double skew;

    private NetworkTableEntry tx;
    private NetworkTableEntry ty;
    private NetworkTableEntry ta;
    private NetworkTableEntry ts;

    public VisionSubsystem(){

//Creates the network tables for use
        NetworkTable limeTable = NetworkTableInstance.getDefault().getTable("limelight");
        tx = limeTable.getEntry("tx");
        ty = limeTable.getEntry("ty");
        ta = limeTable.getEntry("ta");
        ts = limeTable.getEntry("ts");

// creates the logs
        XOffsetLog = new DoubleLogEntry(DataLogManager.getLog(), "/VisionSubsystem/XOffset");
        YOffsetLog = new DoubleLogEntry(DataLogManager.getLog(), "/VisionSubsystem/YOffset");
        AreaLog = new DoubleLogEntry(DataLogManager.getLog(), "/VisionSubsystem/AreaLog");
        SkewLog = new DoubleLogEntry(DataLogManager.getLog(), "/VisionSubsystem/SkewLog");

// creates a shuffleboard tab for this subsystem
        ShuffleboardTab visionTab = Shuffleboard.getTab("Vision Tab");

// creates the collums for the shuffleboard
        visionTab.addNumber("X Offset", () -> x);
        visionTab.addNumber("Y Offset", () -> y);
        visionTab.addNumber("Area", () -> area);
        visionTab.addNumber("Skew", () -> skew);
     }

     @Override
     public void periodic() {

// continually puts the values in the log
        XOffsetLog.append(x);
        YOffsetLog.append(y);
        AreaLog.append(area);
        SkewLog.append(skew);

// continually updates the network tables
        skew = ts;
        x = tx;
        y = ty;
        area = ta;

     }
   
}
