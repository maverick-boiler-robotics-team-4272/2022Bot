package frc.robot.Subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

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
        return table.getEntry("ty").getDouble(1.0);
    }

    public static double getTY(){
        return table.getEntry("tx").getDouble(0.0);
    }

    public static double getDistanceFeet(){
        return (Constants.GOAL_HEIGHT - Constants.LIMELIGHT_HEIGHT) / Math.tan((SmartDashboard.getNumber("Limelight Ground Angle", 0.0) + Limelight.getTY()) * Math.PI / 180.0);
    }

    public static void setLEDMode(LEDMode mode){
        boolean light;
        if(mode.equals(LEDMode.OFF)){
            light = false;
        }else{
            light = true;
        }

        SmartDashboard.putBoolean("Limelight light", light);

        table.getEntry("ledMode").setNumber(mode.tableVal);
    }

    public static boolean isValidTarget(){
        return ((int) table.getEntry("tv").getDouble(0.0)) != 0;
    }

    public static boolean getAimed(){
        return (Math.abs(Limelight.getTX()) < Constants.LIMELIGHT_DEADZONE && Limelight.isValidTarget());
    }
}