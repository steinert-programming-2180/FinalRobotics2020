package frc.robot;

import java.util.HashMap;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class UniversalVar{
    HashMap<String, Object> universalHash = new HashMap<String, Object>();
    NetworkTableInstance netInst;
    NetworkTable table;

    UniversalVar(){
        netInst = NetworkTableInstance.getDefault();
        table = netInst.getTable("datatable");
    }

    public void writeToTable(){
        for(String i : universalHash.keySet()){
            table.getEntry(i).setValue(universalHash.get(i));
        }
    }

    public Object getVal(String key){
        return universalHash.get(key);
    }
    
    public void add(String key, Object value){
        universalHash.put(key, value);
    }
}