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

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Units;

import static frc.robot.Constants.ShooterConstants;
import static frc.robot.RobotUtilities.*;
import frc.robot.DriveWrapper;
import frc.robot.UniversalVar;

public class Shooter extends SubsystemBase {
  /**
   * Creates a new ExampleSubsystem.
   */
  CANSparkMax[] shooterMotors;
  CANEncoder shooterEncoder;
  CANPIDController shooterPID;
  DriveWrapper shooterWrapper;

  UniversalVar uniVar;

  private double shooterFFVoltage, shooterSpeed, shooterTargetSpeed;

  public Shooter(UniversalVar uniVar) {
    this.uniVar = uniVar;
    shooterMotors = SetUpMotors(ShooterConstants.shooterMotorPorts, ShooterConstants.inversionsShooter);

    shooterEncoder = shooterMotors[0].getEncoder();
    shooterEncoder.setPositionConversionFactor(ShooterConstants.positionConversionFactor); //Rotations can stay, value is 1
    shooterEncoder.setVelocityConversionFactor(ShooterConstants.velocityConversionFactor); //Turns rpm to rps, value is 1/60 * gear ratio
    
    shooterPID = new CANPIDController(shooterMotors[0]);
    setUpPID(shooterPID);

    shooterWrapper = new DriveWrapper(ShooterConstants.ShooterKs,
                                      ShooterConstants.ShooterKv,
                                      ShooterConstants.ShooterKa);

    //this.targetSpeed = 0;
  }

  public void setUpPID (CANPIDController pid) {
    pid.setP(ShooterConstants.ShooterKp, 0);
    pid.setI(ShooterConstants.ShooterKi, 0);
    pid.setD(ShooterConstants.ShooterKd, 0);
    pid.setFF(ShooterConstants.ShooterKf, 0); 
    pid.setOutputRange(ShooterConstants.ShooterMin, ShooterConstants.ShooterMax, 0);
  }

  public void shootBall(double speed, Units rotationUnit) {
    switch (rotationUnit) { //Converts to RPS
      case DEGREES:
        speed = speed / 360.0;
        break;
      case RADIANS:
        speed = speed / (2 * Math.PI);
        break;
      case PERCENT:
        speed = speed * ShooterConstants.maximumVelocity;
        break;
    }

    shooterFFVoltage = shooterWrapper.calculateFeedForward(speed);
    shooterPID.setReference(/*speed*/0, ControlType.kVelocity, 0, shooterFFVoltage);
    // shooterMotors[0].setVoltage(shooterFFVoltage);

    // shooterMotors[0].set(speed / ShooterConstants.maximumVelocity);
  }

  public void shootBall(){ //Conveinence wrapper for shooting slam-aligned
    shootBall(ShooterConstants.slamAlignedShotSpeed, Units.ROTATIONS);
    // shooterMotors[0].set(1);
  }

  public void stopShooting() {
    shooterMotors[0].set(0.0);
  }

  public double getSpeed(){
    return this.shooterSpeed;
  }

  public void grabSensors() {
    this.shooterSpeed = shooterEncoder.getVelocity();
    uniVar.add("Shooter-Speed", shooterSpeed);
  }
  
  
  @Override
  public void periodic() {
    grabSensors();
    SmartDashboard.putNumber("Speed", shooterEncoder.getVelocity());
    // SmartDashboard.putNumber("Target", this.targetSpeed);
    // SmartDashboard.putNumber("Error", this.error);
    SmartDashboard.putNumber("Factor", shooterEncoder.getVelocityConversionFactor());
    SmartDashboard.putNumber("Applied Out", shooterMotors[0].getAppliedOutput() * 
                              shooterMotors[0].getBusVoltage());
  }
}