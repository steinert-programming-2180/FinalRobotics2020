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
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpiutil.math.MathUtil;

import static frc.robot.Constants.DrivetrainConstants;
import static frc.robot.Constants.Units;
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
  private DifferentialDriveKinematics kinematicsCalc;
  private DifferentialDriveWheelSpeeds internalWheelSpeeds;
  private ChassisSpeeds internalChassis;
  
  private AHRS navX;
  private PIDController anglePid;

  private double leftFFVoltage = 0;
  private double rightFFVoltage = 0;

  private long previousTime = 0;
  private double previousDesireVel = 0;
  
  private double leftPosition, leftVelocity, rightPosition, rightVelocity, //Grab from encoders, linear
          chassisVelocity, chassisPosition, chassisAccelleration, chassisAngle, rotVelocity; //Grab from NavX
  
  public Drivetrain() {
    leftMotors = SetUpMotors(DrivetrainConstants.leftMotorPorts, DrivetrainConstants.inversionsLeft); //All motor stuff
    rightMotors = SetUpMotors(DrivetrainConstants.rightMotorPorts, DrivetrainConstants.inversionsRight);
    kinematicsCalc = new DifferentialDriveKinematics(DrivetrainConstants.effectiveDrivebaseWidth);

    leftEncoder = leftMotors[0].getEncoder();
    leftEncoder.setPositionConversionFactor(DrivetrainConstants.positionConversionFactor); //Converts to meters and meters/sec
    leftEncoder.setVelocityConversionFactor(DrivetrainConstants.velocityConversionFactor);
    rightEncoder = rightMotors[0].getEncoder();
    rightEncoder.setVelocityConversionFactor(DrivetrainConstants.velocityConversionFactor);
    rightEncoder.setPositionConversionFactor(DrivetrainConstants.positionConversionFactor);

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
    linearPID.setOutputRange(DrivetrainConstants.Min[index], DrivetrainConstants.Max[index]);
  }

  public void setDrive(double leftSpeed, double rightSpeed, Units lengthUnit){ //Pure differential drive
    switch (lengthUnit) { //Permits the use of all kinds of units.  Internally still working with m/s though
      case INCHES: 
        leftSpeed = leftSpeed / DrivetrainConstants.conversionFactor; 
        rightSpeed = rightSpeed / DrivetrainConstants.conversionFactor;
        break;
      case ROTATIONS: 
        leftSpeed = leftSpeed * DrivetrainConstants.gearRatio * DrivetrainConstants.wheelDiameter * Math.PI;
        rightSpeed = rightSpeed * DrivetrainConstants.gearRatio * DrivetrainConstants.wheelDiameter * Math.PI;
        break;
    }

    leftLinearPid.setReference(leftSpeed, ControlType.kVelocity);
    rightLinearPid.setReference(rightSpeed, ControlType.kVelocity);
  } 
  
  //This version just uses Chassis classes to convert from velocity and rotational terms to two velocity terms, and passes them to the 
  //OG setDrive.
  public void setDrive (double speed, double rotationalVelocity, Units lengthUnit, Units rotationUnit) { 
    switch (lengthUnit) { //Allows for multiple units, saddly poorly compressable
      case INCHES:
        speed = speed / DrivetrainConstants.conversionFactor;
        break;
      case ROTATIONS:
        speed = speed * DrivetrainConstants.gearRatio * DrivetrainConstants.wheelDiameter * Math.PI;
        break;
    }

    switch (rotationUnit) {
      case ROTATIONS:
        rotationalVelocity = rotationalVelocity / (Math.PI * 2);
        break;
      case DEGREES:
        rotationalVelocity = rotationalVelocity * (Math.PI / 180);
        break;
    }

    internalChassis = new ChassisSpeeds(speed, 0, rotationalVelocity);
    internalWheelSpeeds = kinematicsCalc.toWheelSpeeds(internalChassis);
    this.setDrive(internalWheelSpeeds.leftMetersPerSecond, internalWheelSpeeds.rightMetersPerSecond, Units.METERS);
  }

  public void turnToAngle(double angleInDegrees){
    double pidVal = MathUtil.clamp(anglePid.calculate(navX.getAngle()), -1, 1);

    //What happens initially, since actualTurnToAngle would setDrive with 0
    if(!actualTurnToAngle()){
      actualTurnToAngle();
    } else{
      anglePid.setSetpoint(navX.getAngle() + angleInDegrees);
    }
  }

  public boolean actualTurnToAngle(){
    double pidVal = MathUtil.clamp(anglePid.calculate(navX.getAngle()), -1, 1);
    setDrive(pidVal, pidVal, Units.METERS);
    if(pidVal > 0.05){
      return false;
    } else{
      return true;
    }
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

  double getPIDAcceleration(double currentDesireVel){
    double changeInVelocity = currentDesireVel-previousDesireVel;
    previousDesireVel = currentDesireVel;
    double changeInTime = System.currentTimeMillis() - previousTime;
    previousTime = System.currentTimeMillis(); 

    return changeInVelocity/changeInTime;
  }

  @Override
  public void periodic() {
    grabSensors();
  }
}
