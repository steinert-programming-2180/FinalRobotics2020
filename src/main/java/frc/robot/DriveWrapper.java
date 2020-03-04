package frc.robot;

import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;

public class DriveWrapper {
    SimpleMotorFeedforward ffCalc;
    double previousTime = -1;
    double previousSpeed = -1;
    double acceleration;
    public DriveWrapper (double kS, double kV, double kA) {
        ffCalc = new SimpleMotorFeedforward(kS, kV, kA);
    }

    public double calculateFeedForward (double newTime, double newSpeed) {
        if (previousTime != -1 && previousSpeed != -1) {
            acceleration = (newSpeed - previousSpeed) / (newTime - previousTime);
            previousTime = newTime;
            previousSpeed = newSpeed;
            return ffCalc.calculate(newSpeed, acceleration);
        } else {
            previousTime = newTime;
            previousSpeed = newSpeed;
            return this.calculateFeedForward(newSpeed);
        }
    } public double calculateFeedForward (double speed) {
        return ffCalc.calculate(speed);
    }

    public void resetRun () {
        previousTime = -1;
        previousSpeed = -1;
    }
}