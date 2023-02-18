package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.datalog.BooleanLogEntry;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;

public class VisionSubsystem extends SubsystemBase{

    private static BooleanLogEntry BLog;
    private static DoubleLogEntry DLog;

    private double x;
    private double y;
    private double area;

    public VisionSubsystem(){
        ShuffleboardTab visionTab = Shuffleboard.getTab("Vision Tab");

        visionTab.addNumber("X Offset", () -> x);
        visionTab.addNumber("Y Offset", () -> y);
        visionTab.addNumber("Area", () -> area);
     }

     @Override
     public void periodic() {
        NetworkTable limeTable = NetworkTableInstance.getDefault().getTable("limelight");
        NetworkTableEntry tx = limeTable.getEntry("tx");
        NetworkTableEntry ty = limeTable.getEntry("ty");
        NetworkTableEntry ta = limeTable.getEntry("ta");

        x = tx.getDouble(0.0);
        y = ty.getDouble(0.0);
        area = ta.getDouble(0.0);

     }
   
}
