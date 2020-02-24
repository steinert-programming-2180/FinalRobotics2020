package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class RobotUtilities {
    public static CANSparkMax[] SetUpMotors(int[] ports, boolean[] inverts) {
        CANSparkMax[] output = new CANSparkMax[ports.length];
        output[0] = new CANSparkMax(ports[0], MotorType.kBrushless);

        for (int i = 1; i < ports.length; i++) {
            output[i] = new CANSparkMax(ports[i], MotorType.kBrushless);
            output[i].follow(output[0], inverts[i]);
        }

        return output;
    }

    public static double inchesToMeters (double inches) {
        return 0.0254 * inches;
    }
}