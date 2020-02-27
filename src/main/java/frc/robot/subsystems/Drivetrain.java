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
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
  private SimpleMotorFeedforward leftFeedForward, rightFeedForward;
  private AHRS navX;
  private PIDController anglePid = new PIDController(
    DrivetrainConstants.AngleKp,
    DrivetrainConstants.AngleKi, 
    DrivetrainConstants.AngleKd);

  private double leftFFVoltage = 0;
  private double rightFFVoltage = 0;

  private long previousTime = 0;
  private double previousDesireVel = 0;
  
  private double leftPosition, leftVelocity, rightPosition, rightVelocity, //Grab from encoders, linear
          chassisVelocity, chassisPosition, chassisAccelleration, chassisAngle, rotVelocity; //Grab from NavX
  
  public Drivetrain() {
    leftMotors = SetUpMotors(DrivetrainConstants.leftMotorPorts, DrivetrainConstants.inversionsLeft); //All motor stuff
    rightMotors = SetUpMotors(DrivetrainConstants.rightMotorPorts, DrivetrainConstants.inversionsRight);

    leftEncoder = leftMotors[0].getEncoder();
    rightEncoder = rightMotors[0].getEncoder();

    navX = new AHRS(SPI.Port.kMXP);
    anglePid = new PIDController(DrivetrainConstants.AngleKp, DrivetrainConstants.AngleKi,DrivetrainConstants.AngleKd);

    leftLinearPid = new CANPIDController(leftMotors[0]);
    rightLinearPid = new CANPIDController(rightMotors[0]);

    leftFeedForward = new SimpleMotorFeedforward(DrivetrainConstants.Ks[0], DrivetrainConstants.Kv[0]);
    rightFeedForward = new SimpleMotorFeedforward(DrivetrainConstants.Ks[1], DrivetrainConstants.Kv[1]);

    setupPID(leftLinearPid, leftFFVoltage, true);
    setupPID(rightLinearPid, rightFFVoltage, false);
  }

  void setupPID(CANPIDController linearPID, double FF, boolean isLeft){
    int index = isLeft ? 0:1;

    linearPID.setP(DrivetrainConstants.Kp[index]);
    linearPID.setI(DrivetrainConstants.Ki[index]);
    linearPID.setD(DrivetrainConstants.Kd[index]);
    linearPID.setIZone(DrivetrainConstants.Izone[index]);
    linearPID.setFF(FF);
    linearPID.setOutputRange(DrivetrainConstants.Min[index], DrivetrainConstants.Max[index]);
  }

  double getPIDAcceleration(double currentDesireVel){
    double changeInVelocity = currentDesireVel-previousDesireVel;
    previousDesireVel = currentDesireVel;
    double changeInTime = System.currentTimeMillis() - previousTime;
    previousTime = System.currentTimeMillis(); 

    return changeInVelocity/changeInTime;
  }

  public void setDrive(double leftSpeed, double rightSpeed){
      //These recalculate the feedforward whenever called.  The constants are in units of (Volt * Rotations) / S
      leftFFVoltage = leftFeedForward.calculate(leftSpeed / 60);
      rightFFVoltage = rightFeedForward.calculate(rightSpeed / 60);
      leftLinearPid.setFF(leftFFVoltage / leftMotors[0].getBusVoltage());
      rightLinearPid.setFF(rightFFVoltage / rightMotors[0].getBusVoltage());
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

    leftLinearPid.setFF(leftFFVoltage / leftMotors[0].getBusVoltage());
    rightLinearPid.setFF(rightFFVoltage / rightMotors[0].getBusVoltage());
  }

  @Override
  public void periodic() {
    grabSensors();
    SmartDashboard.putNumber("Actual Left" ,this.leftVelocity);
    SmartDashboard.putNumber("Actual Right" ,this.rightVelocity);
  }
}
