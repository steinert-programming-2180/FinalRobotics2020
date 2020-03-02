/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * Conventions of constants:
 * -All constants should be named specifically, even in a single subsystem, to allow for new features to be added.
 * -Generally, objects should be declared where they will be used.  Even if there is a 1:1 mapping of constants to 
 *   objects (ie. motor ports to motors), keeping the objects elsewhere makes Constants.java more extensible.
 * -In the same vein, it's better to declare too many constants than too few, but redundancy is the enemy.  Nothing 
 *   should EVER be declared twice.
 * -Even if only a single motor is expected, declare its port and inversion in an array.
 * -The first port in a motor port array will be made the leader.
 * -All the motors created from a single motor port array will follow their leader.  If motors need to be controlled
 *   independently from each other, they should be placed in seperate arrays.
 * -Inversion arrays should be located directly below the port array they will be used on.  They are applied in order
 *   to the ports.  The first value is the inversion of the leader, and the rest will dictate inversion RELATIVE TO 
 *   THE LEADER.  
 * -In all cases where it isn't obvious, go top-to-down and left-to-right, one area at a time.  Remember to comment.
 */
public final class Constants {
    public static class IOPorts{
        public static int[] beamSensors = {0};

        public static int controllerPort = 0;
    }
    public static class DrivetrainConstants{
        public static int[] leftMotorPorts = {1, 2, 3};
        public static boolean[] inversionsLeft = {false, false, false};
        public static int[] rightMotorPorts = {4, 5, 6};
        public static boolean[] inversionsRight = {true, false, false};
   
        public static double AngleKp = 0.0;
        public static double AngleKi = 0.0;
        public static double AngleKd = 0.0;

        //Variable = {LeftValue, RightValue}
        public static double[] Kp = {0.0, 0.0};
        public static double[] Ki = {0.0, 0.0};
        public static double[] Kd = {0.0, 0.0};
        public static double[] Izone = {0.0, 0.0};
        public static double[] Min = {-1.0, -1.0};
        public static double[] Max = {1.0, 1.0};
        public static double[] Accel = {0.0, 0.0};
        public static double[] MaxVel = {0.0, 0.0};
        public static double[] Ks = {0.106, 0.137};
        public static double[] Kv = {0.0679, 0.0712};
        public static double[] Ka = {0.00675, 0.0141};

        public static double defaultMotorFactor = 0.4;

        public static double effectiveDrivebaseWidth = 0; //Should be in inches
        public static double rotationRatio = 1; //Number of rotations of final wheel:Number of rotations of motor
        public static double wheelDiameter = 3; //Should be in inches
    }
    public static class PaddyConstants{
        public static int[] turnerMotors = {12};
        public static boolean[] inversionsTurner = {false};

        public static double[] blueVals = {0.232154, 0.427, 0.429};
        public static double[] greenVals = {0.197, 0.561, 0.240};
        public static double[] redVals = {0.291504, 0.322754, 0.110107};
        public static double[] yellowVals = {0.322266, 0.571777, 0.105957};
    }
    public static class ShooterConstants {
        public static int[] shooterMotorPorts = {7, 8};
        public static boolean[] inversionsShooter = {true, true};

        public static double ShooterKp = 0.0;
        public static double ShooterKi = 0.0;
        public static double ShooterKd = 0.0;
    }
    public static class ClimberConstants{
        public static int[] elevatorMotorPorts = {12, 13};
        public static boolean[] inversionsElevator = {false, true};
    }
}
