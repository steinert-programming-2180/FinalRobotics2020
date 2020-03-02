/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.ShooterConstants;
import static frc.robot.RobotUtilities.*;

public class Shooter extends SubsystemBase {
  /**
   * Creates a new ExampleSubsystem.
   */
  CANSparkMax[] shooterMotors;
  CANEncoder shooterEncoder;
  CANPIDController shooterPID;

  double speed;

  public Shooter() {
     shooterMotors = SetUpMotors(ShooterConstants.shooterMotorPorts, ShooterConstants.inversionsShooter);

     shooterEncoder = shooterMotors[0].getEncoder();
     shooterEncoder.setPositionConversionFactor(ShooterConstants.positionConversionFactor); //Rotations can stay, value is 1
     shooterEncoder.setPositionConversionFactor(ShooterConstants.velocityConversionFactor); //Turns rpm to rps, value is 1/60

     shooterPID = new CANPIDController(shooterMotors[0]);
     setUpPID(shooterPID);
  }

  public void setUpPID (CANPIDController pid) {
    pid.setP(ShooterConstants.ShooterKp);
    pid.setI(ShooterConstants.ShooterKi);
    pid.setD(ShooterConstants.ShooterKd);
    pid.setOutputRange(ShooterConstants.ShooterMin, ShooterConstants.ShooterMax);
  }

  public void shootBall(double speed) {
    shooterPID.setReference(speed, ControlType.kVelocity);
  }

  public void shootBall(){ //Conveinence wrapper for shooting slam-aligned
    shootBall(ShooterConstants.slamAlignedShotSpeed);
  }

  public void stopShooting() {
    shooterMotors[0].set(0.0);
  }

  public void grabSensors() {
    this.speed = shooterEncoder.getVelocity();
  }

  @Override
  public void periodic() {
    grabSensors();
  }
}
