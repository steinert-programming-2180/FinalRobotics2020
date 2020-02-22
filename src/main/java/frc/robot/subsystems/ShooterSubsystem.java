/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {
  /**
   * Creates a new ExampleSubsystem.
   */
  CANSparkMax mainShooterLeft;
  CANSparkMax mainShooterRight;
  CANSparkMax[] ballUpMotors = new CANSparkMax[ShooterConstants.shooterBallUpPorts.length];

  public ShooterSubsystem() {
      mainShooterLeft = new CANSparkMax(ShooterConstants.shooterMotorPorts[0], MotorType.kBrushless);
      mainShooterRight = new CANSparkMax(ShooterConstants.shooterMotorPorts[1], MotorType.kBrushless);
  }

  public void shootBall(){
      mainShooterLeft.set(-1);
      mainShooterRight.set(1);
  }

  public void shootBall(double speed) {}

  public void stopShooting() {
    mainShooterLeft.set(0);
    mainShooterRight.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
