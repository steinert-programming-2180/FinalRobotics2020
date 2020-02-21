/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.SPI;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants.  This class should not be used for any other purpose.  All constants should be
 * declared globally (i.e. public static).  Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    CANSparkMax left1 = new CANSparkMax(1, MotorType.kBrushless);
    CANSparkMax left2 = new CANSparkMax(2, MotorType.kBrushless);
    CANSparkMax left3 = new CANSparkMax(3, MotorType.kBrushless);

    CANSparkMax right1 = new CANSparkMax(4, MotorType.kBrushless);
    CANSparkMax right2 = new CANSparkMax(5, MotorType.kBrushless);
    CANSparkMax right3 = new CANSparkMax(6, MotorType.kBrushless);

    AHRS ahrs = new AHRS(SPI.Port.kMXP);
}
