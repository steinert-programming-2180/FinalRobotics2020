package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class RobotUtilities {
    public static CANSparkMax[] SetUpMotors(int[] ports) {
        CANSparkMax[] output = new CANSparkMax[ports.length];
        CANSparkMax dummyMotor;
        int counter = 0;
        for (int i : ports) {
            dummyMotor = new CANSparkMax(i, MotorType.kBrushless);
            output[counter] = dummyMotor;
            counter++;
        } return output;
    }
}