import java.util.HashMap;

import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Conveyer;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Funnel;
import frc.robot.subsystems.Shooter;

public class UniversalVar{
    HashMap<String, HashMap<String, Object>> universalHash = new HashMap<String, HashMap<String, Object>>();
    Drivetrain drive; Climber climb; Conveyer conveyer; Funnel funnel; Shooter shooter;

    UniversalVar(Drivetrain drive, Climber climb, Conveyer conveyer, Funnel funnel, Shooter shooter){
        this.drive = drive;
        this.climb = climb;
        this.conveyer = conveyer;
        this.funnel = funnel;
        this.shooter = shooter;
        generalUpdate();
    }

    public void generalUpdate(){
        updateClimber(climb.getPosition(), climb.getVelocity());
        updateConveyor(conveyer.getTopConveyerBeam(), conveyer.getBottomConveyerBeam(), conveyer.getConveyerPosition(), conveyer.getConveyerVelocity());
        updateDrivetrain(drive.getLeftSpeed(), drive.getLeftPosition(), drive.getRightSpeed(), drive.getRightPosition(), drive.getAngle(), drive.getPosition(), drive.getChassisVelocity(), drive.getAcceleration(), drive.getAngularVelocity());
        updateFunnel(funnel.getFunnelBeam(), funnel.getFunnelPosition(), funnel.getFunnelVelocity());
        updateShooter(shooter.getSpeed());
    }

    public Object getVal(String key){
        return universalHash.get(key);
    }
    
    public void updateClimber(double position, double velocity){
        if(universalHash.keySet().contains("Climber")){
            HashMap<String, Object> current = universalHash.get("Climber");
            current.put("Position", position);
            current.put("Velocity", velocity);
            //{"Climber": }
        } else{
            universalHash.put("Climber", new HashMap<String, Object>());
        }
    }

    public void updateConveyor(boolean top, boolean bottom, double position, double velocity){
        if(universalHash.keySet().contains("Conveyor")){
            HashMap<String, Object> current = universalHash.get("Conveyor");
            current.put("Top", top);
            current.put("Bottom", bottom);
            current.put("Position", position);
            current.put("Velocity", velocity);
        } else{
            universalHash.put("Conveyor", new HashMap<String, Object>());
        }
    }

    public void updateDrivetrain(double lVel, double lPos, double rVel, double rPos, double chasAng, double chasPos, double chasVel, double chasAcel, double rotVel){
        if(universalHash.keySet().contains("Drivetrain")){
            HashMap<String, Object> current = universalHash.get("Drivetrain");
            current.put("Left Velocity", lVel);
            current.put("Left Position", lPos);
            current.put("Right Velocity", rVel);
            current.put("Right Position", rPos);
            current.put("Chassis Angle", chasAng);
            current.put("Chassis Position", chasPos);
            current.put("Chassis Velocity", chasVel);
            current.put("Chassis Accelleration", chasAcel);
            current.put("Rotational Velocity", rotVel);
        } else{
            universalHash.put("Drivetrain", new HashMap<String, Object>());
        }
    }

    public void  updateFunnel(boolean beam, double position, double velocity){
        if(universalHash.keySet().contains("Funnel")){
            HashMap<String, Object> current = universalHash.get("Funnel");
            current.put("Beam", beam);
            current.put("Position", position);
            current.put("Velocity", velocity);
        } else{
            universalHash.put("Funnel", new HashMap<String, Object>());
        }
    }

    public void updateShooter(double velocity){
        if(universalHash.keySet().contains("Shooter")){
            HashMap<String, Object> current = universalHash.get("Shooter");
            current.put("Velocity", velocity);
        } else{
            universalHash.put("Shooter", new HashMap<String, Object>());
        }
    }
}