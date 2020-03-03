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
 * -Distance units can be anything, so long as you note it in a comment, and add it to the enum so it is self documenting.  
 *   Derivatives thereof, however, MUST be with respect to SECONDS (ie. with inches velocity would by in/sec and with meters
 *   acceleration would be m/sec^2).  Yes sparks run rpm, yes I only want it in seconds because good ol Letson trained me to
 *   expect it.
 */
public final class Constants {
    public static enum Units {
        METERS,
        INCHES,
        DEGREES,
        RADIANS,
        ROTATIONS,
        PERCENT
    }
    public static class IOPorts{
        public static int[] beamSensors = {0,1,2,3,4,5};

        public static int[] driverPorts = {0}; //Controls go from left to right
        public static int[] operatorPorts = {1}; 
    }

    public static class DrivetrainConstants{
        public static int[] leftMotorPorts = {1, 2, 3};
        public static boolean[] inversionsLeft = {false, false, false};
        public static int[] rightMotorPorts = {4, 5, 6};
        public static boolean[] inversionsRight = {true, false, false};

        public static double conversionFactor = 39.3701;
        //2371 on motor for shooter
   
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

        public static double effectiveDrivebaseWidth = 0; //Should be in meters
        public static double gearRatio = 1; //Number of rotations of final wheel:Number of rotations of motor
        public static double wheelDiameter = 3; //Should be in meters
        public static double positionConversionFactor = gearRatio * wheelDiameter;
        public static double velocityConversionFactor = positionConversionFactor / 60.0;
        public static double maximumVelocity = 10;
    }
    
    public static class PaddyConstants{
        public static int[] turnerMotors = {12};
        public static boolean[] inversionsTurner = {false};

        public static double PaddyKp = 0.0;
        public static double PaddyKi = 0.0;
        public static double PaddyKd = 0.0;
        public static double PaddyMin = -1;
        public static double PaddyMax = 1;
        public static double PaddyKs = 0;
        public static double PaddyKv = 0;
        public static double PaddyKa = 0;

        public static double[] blueVals = {0.232154, 0.427, 0.429};
        public static double[] greenVals = {0.197, 0.561, 0.240};
        public static double[] redVals = {0.291504, 0.322754, 0.110107};
        public static double[] yellowVals = {0.322266, 0.571777, 0.105957};
    }
    public  class customController{
         int shooterPort = 0;
         int paddyPort = 0;
         int intake = 0; 
         int climber = 0;
         
            }
    
    public static class ShooterConstants {        
        public static int[] shooterMotorPorts = {7, 8};
        public static boolean[] inversionsShooter = {true, true};

        public static double ShooterKp = 0.0;
        public static double ShooterKi = 0.0;
        public static double ShooterKd = 0.0;
        public static double ShooterMin = -1;
        public static double ShooterMax = 1;
        public static double ShooterKs = 0;
        public static double ShooterKv = 0;
        public static double ShooterKa = 0;

        public static double gearRatio = 24.0 / 18.0;
        public static double positionConversionFactor = 1;
        public static double velocityConversionFactor = 1.0 / 60.0;
        public static double maximumVelocity = 1;

        public static double slamAlignedShotSpeed = 35;
    }

    public static class ClimberConstants{
        public static int[] elevatorMotorPorts = {12, 13};
        public static boolean[] inversionsElevator = {false, true};
    }

    public static class IntakeConstants{
        public static class Conveyer {
            public static int[] motorPorts = {0};
            public static boolean[] motorInversions = {false};

            public static double gearRatio = 1; //Expressed in terms of final rotations over motor rotations
            public static double driveGearDiameter = 1;
            public static double positionConversionFactor = driveGearDiameter * Math.PI * 2 * gearRatio;
            public static double velocityConversionFactor = positionConversionFactor / 60;
            public static double maxSpeed = 0;
        } public static class Funnel {
            public static int[] motorPorts = {1};
            public static boolean[] motorInversions = {false};

            public static double gearRatio = 1; //Expressed in terms of final rotations over motor rotations
            public static double driveGearDiameter = 1;
            public static double positionConversionFactor = driveGearDiameter * Math.PI * 2 * gearRatio;
            public static double velocityConversionFactor = positionConversionFactor / 60;
            public static double maxSpeed = 0;
        } public static class Intake {
            public static int[] motorPorts = {0, 1};
            public static boolean[] motorInversions = {false};

            public static double gearRatio = 1; //Expressed in terms of final rotations over motor rotations
            public static double driveGearDiameter = 1;
            public static double positionConversionFactor = driveGearDiameter * Math.PI * 2 * gearRatio;
            public static double velocityConversionFactor = positionConversionFactor / 60;
            public static double maxSpeed = 0;
        }
        public static int[] beamTripPorts = {0, 1, 2};
    }
}
