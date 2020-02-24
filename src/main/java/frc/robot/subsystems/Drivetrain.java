/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.DrivetrainConstants;
import static frc.robot.RobotUtilities.*;

public class Drivetrain extends SubsystemBase {
  /**
   * Creates a new ExampleSubsystem.
   */
  private CANSparkMax[] leftMotors;
  private CANSparkMax[] rightMotors;
  private CANEncoder leftEncoder, rightEncoder;
  private CANPIDController leftLinearPid, rightLinearPid;
  private AHRS navX;
  private PIDController anglePid = new PIDController(DrivetrainConstants.AngleKp,
                                                    DrivetrainConstants.AngleKi, 
                                                    DrivetrainConstants.AngleKd);

  private double leftPosition, leftVelocity, rightPosition, rightVelocity, //Grab from encoders, linear
          chassisVelocity, chassisPosition, chassisAccelleration, chassisAngle, rotVelocity; //Grab from NavX
  
  public Drivetrain() {
    leftMotors = SetUpMotors(DrivetrainConstants.leftMotorPorts, DrivetrainConstants.inversionsLeft); //All motor stuff
    rightMotors = SetUpMotors(DrivetrainConstants.rightMotorPorts, DrivetrainConstants.inversionsRight);

    navX = new AHRS(SPI.Port.kMXP);
    anglePid = new PIDController(DrivetrainConstants.AngleKp,
                                  DrivetrainConstants.AngleKi, 
                                  DrivetrainConstants.AngleKd);
    
    leftLinearPid = new CANPIDController(leftMotors[0]);
    leftLinearPid.setP(DrivetrainConstants.LeftKp);
    leftLinearPid.setI(DrivetrainConstants.LeftKi);
    leftLinearPid.setD(DrivetrainConstants.LeftKd);
    leftLinearPid.setIZone(DrivetrainConstants.LeftIzone);
    leftLinearPid.setFF(DrivetrainConstants.LeftKf);
    leftLinearPid.setOutputRange(DrivetrainConstants.LeftMin, DrivetrainConstants.LeftMax);
    rightLinearPid = new CANPIDController(rightMotors[0]);
    rightLinearPid.setP(DrivetrainConstants.RightKp);
    rightLinearPid.setI(DrivetrainConstants.RightKi);
    rightLinearPid.setD(DrivetrainConstants.RightKd);
    rightLinearPid.setIZone(DrivetrainConstants.RightIzone);
    rightLinearPid.setFF(DrivetrainConstants.RightKf);
    rightLinearPid.setOutputRange(DrivetrainConstants.RightMin, DrivetrainConstants.RightMax);

    leftEncoder = leftMotors[0].getEncoder();
    rightEncoder = rightMotors[0].getEncoder();
  }

   void setDrive(double leftSpeed, double rightSpeed){
      leftLinearPid.setReference(leftSpeed, ControlType.kVelocity);
      rightLinearPid.setReference(rightSpeed, ControlType.kVelocity);
  }

  public double getAngularVelocity(){
    return this.rotVelocity;
  }

  public double getAngle(){
    return this.chassisAngle;
  }

  public double getAcceleration(){
    return this.chassisAccelleration;
  }

  public double getLeftSpeed() {
    return this.leftVelocity;
  }

  public double getRightSpeed() {
    return this.rightVelocity;
  }

  public void grabSensors() {
    this.leftPosition = this.leftEncoder.getPosition();
    this.leftVelocity = this.leftEncoder.getVelocity();
    this.rightPosition = this.rightEncoder.getPosition();
    this.rightVelocity = this.rightEncoder.getVelocity();
    
    this.chassisAngle = this.navX.getAngle();
    this.chassisPosition = this.navX.getDisplacementX();
    this.chassisVelocity = this.navX.getVelocityX();
    this.chassisAccelleration = this.navX.getRawAccelX();
    this.rotVelocity = this.navX.getRawGyroZ();
  }

  @Override
  public void periodic() {
    grabSensors();
  }
}
