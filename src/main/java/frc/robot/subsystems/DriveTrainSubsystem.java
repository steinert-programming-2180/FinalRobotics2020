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
import frc.robot.Constants;

public class DriveTrainSubsystem extends SubsystemBase {
  /**
   * Creates a new ExampleSubsystem.
   */

    Constants c = new Constants();

    CANSparkMax[] leftMotors = new CANSparkMax[c.leftMotorPorts.length];
    CANSparkMax[] rightMotors = new CANSparkMax[c.rightMotorPorts.length];

    double Kp = 0.0;
    double Ki = 0.0;
    double Kd = 0.0;
    PIDController gyroPid = new PIDController(Kp, Ki, Kd);
    
  public DriveTrainSubsystem() {
    for(int i = 0; i < c.leftMotorPorts.length; i++){
        leftMotors[i] = new CANSparkMax(c.leftMotorPorts[i], MotorType.kBrushless);
    }

    for(int i = 0; i < c.rightMotorPorts.length; i++){
        rightMotors[i] = new CANSparkMax(c.rightMotorPorts[i], MotorType.kBrushless);
    }
    setSparkFollows();
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

  public void setDrive(double leftSpeed, double rightSpeed){
      leftMotors[0].set(leftSpeed*c.motorFactor);
      rightMotors[0].set(rightSpeed*c.motorFactor);
  }

  public double getVelocity(){
      return c.ahrs.getRawAccelX();
  }

  public double getAngle(){
      return c.ahrs.getAngle();
  }

  public double getAcceleration(){
      return c.ahrs.getRawAccelX();
  }

  public void turnToAngle(double angle){
      if(angle < 0){
        gyroPid.setSetpoint(360+angle);
      } else{
        gyroPid.setSetpoint(angle);
      }
      double value = MathUtil.clamp(gyroPid.calculate(c.ahrs.getAngle()), -1, 1);
      setDrive(value, value);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
