/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpiutil.math.MathUtil;
import static frc.robot.Constants.DrivetrainConstants;

public class Drivetrain extends SubsystemBase {
  /**
   * Creates a new ExampleSubsystem.
   */
  CANSparkMax[] leftMotors = new CANSparkMax[DrivetrainConstants.leftMotorPorts.length];
  CANSparkMax[] rightMotors = new CANSparkMax[DrivetrainConstants.leftMotorPorts.length];

  PIDController gyroPid = new PIDController(DrivetrainConstants.Kp, 
                                            DrivetrainConstants.Ki, 
                                            DrivetrainConstants.Kd);
  
  public Drivetrain() {
    for(int i = 0; i < DrivetrainConstants.leftMotorPorts.length; i++){
        leftMotors[i] = new CANSparkMax(DrivetrainConstants.leftMotorPorts[i], MotorType.kBrushless);
    }

    for(int i = 0; i < DrivetrainConstants.rightMotorPorts.length; i++){
        rightMotors[i] = new CANSparkMax(DrivetrainConstants.rightMotorPorts[i], MotorType.kBrushless);
    }
    setSparkFollows();
  }

  public void setDrive(double leftSpeed, double rightSpeed){
      leftMotors[0].set(leftSpeed * DrivetrainConstants.defaultMotorFactor);
      rightMotors[0].set(rightSpeed * DrivetrainConstants.defaultMotorFactor);
  }

  public double getAngularVelocity(){
    return 0;
  }

  public double getAngle(){
    return 0;
  }

  public double getAcceleration(){
    return 0;
  }

  public double getLeftSpeed() {
    return 0;
  }

  public double getRightSpeed() {
    return 0;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  void setSparkFollows(){
    leftMotors[0].setInverted(true);
    for(int i = 1; i < leftMotors.length; i++){
        leftMotors[i].follow(leftMotors[0], false);
    }

    rightMotors[0].setInverted(false);
    for(int i = 1; i < rightMotors.length; i++){
        rightMotors[i].follow(rightMotors[0], false);
    }
  }
}
