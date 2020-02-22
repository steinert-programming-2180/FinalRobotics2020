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
}