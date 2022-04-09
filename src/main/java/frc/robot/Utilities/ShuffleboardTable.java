package frc.robot.Utilities;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public class ShuffleboardTable {
    
    private final ShuffleboardTab tab;
    private final Map<String, NetworkTableEntry> keyEntryMap = new HashMap<>();

    public ShuffleboardTable(String tabName){
        this.tab = Shuffleboard.getTab(tabName);
        // tab.add("Test", "Hello There").[\]
    }

    private void putEntry(String key, Object value){
        if(!hasKey(key)){
            keyEntryMap.put(key, tab.add(key, value).getEntry());        
        }else{
            keyEntryMap.get(key).setValue(value);
        }
    }

    public boolean hasKey(String key){
        return keyEntryMap.containsKey(key);
    }

    public void putNumber(String key, double value){
        putEntry(key, value);
    }

    public double getNumber(String key, double defaultValue){
        if(!this.hasKey(key)){
            putNumber(key, defaultValue);
            return defaultValue;
        }
        return keyEntryMap.get(key).getDouble(defaultValue);
    }

    public double getNumber(String key){
        return getNumber(key, 0.0);
    }

    public void putBoolean(String key, boolean value){
        putEntry(key, value);
    }

    public boolean getBoolean(String key, boolean defaultValue){
        if(!this.hasKey(key)){
            putBoolean(key, defaultValue);
            return defaultValue;
        }
        return keyEntryMap.get(key).getBoolean(defaultValue);
    }

    public boolean getBoolean(String key){
        return getBoolean(key, false);
    }

    public void putString(String key, String value){
        putEntry(key, value);
    }

    public String getString(String key, String defaultValue){
        if(!this.hasKey(key)){
            putString(key, defaultValue);
            return defaultValue;
        }
        return keyEntryMap.get(key).getString(defaultValue);
    }

    public String getString(String key){
        return getString(key, "");
    }
}
