package frc.robot.Subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Limelight {
    private static final NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");

    public enum LEDMode{
        PIPELINE_CURRENT(0),
        OFF(1),
        BLINK(2),
        ON(3);

        final int tableVal;
        private LEDMode(int tableVal){
            this.tableVal = tableVal;
        }
    };

    public static double getTX(){
        return table.getEntry("tx").getDouble(0.0);
    }

    public static double getTY(){
        return table.getEntry("ty").getDouble(0.0);
    }

    public static double getDistanceFeet(){
        return (8.5 - 29.0 / 12.0) / Math.tan(50.0 + Limelight.getTY());
    }

    public static void setLEDMode(LEDMode mode){
        table.getEntry("ledMode").setNumber(mode.tableVal);
    }
}
