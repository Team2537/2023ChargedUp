package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import frc.robot.Constants;
import frc.robot.Constants.VisionConstants;


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
    private boolean viewedBool;

    private NetworkTableEntry tx;
    private NetworkTableEntry ty;
    private NetworkTableEntry ta;
    private NetworkTableEntry ts;
    private NetworkTableEntry tv;

    private NetworkTable table;

    public VisionSubsystem(){

//Creates the network tables for use
        table = NetworkTableInstance.getDefault().getTable("limelight");
        tx = table.getEntry("tx");
        ty = table.getEntry("ty");
        ta = table.getEntry("ta");
        ts = table.getEntry("ts");
        tv = table.getEntry("tv");
        
        



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
        visionTab.addBoolean("Viewed Target", () -> viewedBool);
        visionTab.addNumber("Current Pipeline", () -> getPipeline());
        visionTab.addNumber("Set Pipeline", () -> getSetPipeline());
        visionTab.addDoubleArray("Position", () -> offsetFromTarget());
     }

     public boolean getIsTargetFound(){
        double v = tv.getDouble(0.0);
        return v != 0.0f;
     }


     public void setPipeline(Integer pipeline) {
        table.getEntry("pipeline").setValue(pipeline);
    }

    public double getPipeline(){
        NetworkTableEntry pipeline = table.getEntry("getpipe");
        double pipe = pipeline.getDouble(0.0);
        return pipe;
    }

    public double getSetPipeline(){
        NetworkTableEntry pipeline = table.getEntry("pipeline");
        double pipe = pipeline.getDouble(0.0);
        return pipe;
    }


     @Override
     public void periodic() {

// continually puts the values in the log
        XOffsetLog.append(x);
        YOffsetLog.append(y);
        AreaLog.append(area);
        SkewLog.append(skew);

// continually updates the network tables
        skew = ts.getDouble(0.0);
        x = tx.getDouble(0.0);
        y = ty.getDouble(0.0);
        area = ta.getDouble(0.0);
        viewedBool = getIsTargetFound();
        

     }

     public double[] offsetFromTarget() {
        if (getPipeline() != 1) return new double[] {-1, -1};

        double heightOffset = Constants.VisionConstants.APRIL_TAG_HEIGHT - Constants.VisionConstants.LIMELIGHT_HEIGHT;
        
        double yOff = heightOffset / Math.tan(Math.toRadians(y));
        double xOff = (heightOffset / Math.sin(Math.toRadians(y))) * Math.tan(Math.toRadians(x));

        return new double[] {xOff, yOff};
     }
   
}