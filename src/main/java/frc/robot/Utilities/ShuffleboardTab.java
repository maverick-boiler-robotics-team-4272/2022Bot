package frc.robot.Utilities;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class ShuffleboardTab {
    
    private final NetworkTable table;

    public ShuffleboardTab(String tabName){
        this.table = NetworkTableInstance.getDefault().getTable(tabName);
    }
}
