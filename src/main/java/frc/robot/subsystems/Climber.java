/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.kauailabs.navx.frc.AHRS;
import static frc.robot.RobotUtilities.*;

import static frc.robot.Constants.ClimberConstants;

public class Climber extends SubsystemBase {
  /**
   * Creates a new Climber.
   */
  CANSparkMax[] elevatorMotors;
  CANEncoder encoder;
  private AHRS navX;
  private double Position, Velocity, //Grab from encoders, linear
  chassisVelocity, chassisPosition, chassisAccelleration, chassisAngle, rotVelocity; //Grab from NavX
   public Climber() {
    elevatorMotors = SetUpMotors(ClimberConstants.elevatorMotorPorts, ClimberConstants.inversionsElevator);
    encoder = elevatorMotors[0].getEncoder();

  }

   public void startClimb(){

   }
   public void endClimb(){

   }
   public double getPosition(){
     return this.Position;
   }
   public double getVelocity(){
     return this.Velocity;
   }
   
   public void grabSensors(){
    this.Position = this.encoder.getPosition();
    this.Velocity = this.encoder.getVelocity();
    
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