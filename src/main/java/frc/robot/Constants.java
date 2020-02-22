/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants.  This class should not be used for any other purpose.  All constants should be
 * declared globally (i.e. public static).  Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static class DrivetrainConstants{
        public static int[] leftMotorPorts = {1, 2, 3};
        public static int[] rightMotorPorts = {4, 5, 6};
   

        public static double Kp = 0.0;
        public static double Ki = 0.0;
        public static double Kd = 0.0;

        public static double defaultMotorFactor = 0.4;
    }
    public static class PaddyConstants{
        public static int pattyMotor = 12;
    }
    public static class ShooterConstants {
        public static int[] shooterMotorPorts = {7, 8};
        public static int[] shooterBallUpPorts = {9, 10, 11};
    }
}
