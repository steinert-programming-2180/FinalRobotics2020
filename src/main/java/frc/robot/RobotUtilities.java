package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class RobotUtilities {
    public static CANSparkMax[] SetUpMotors(int[] ports) {
        CANSparkMax[] output = new CANSparkMax[ports.length];

        for(int i = 0; i < ports.length; i++){
            output[i] = new CANSparkMax(ports[i], MotorType.kBrushless);
        }
        
        return output;
    }

    public static void setSparkFollows(CANSparkMax[] motors, boolean leaderInverted){
        motors[0].setInverted(leaderInverted);
        for(int i = 1; i < motors.length; i++){
            motors[i].follow(motors[0], false);
        }
    }
}