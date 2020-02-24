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
import static frc.robot.RobotUtilities.*;

import static frc.robot.Constants.ClimberConstants;

public class Climber extends SubsystemBase {
  /**
   * Creates a new Climber.
   */
  CANSparkMax[] elevatorMotors;
  CANEncoder encoder;
  private double velocity, position;
  
  public Climber() {
    elevatorMotors = SetUpMotors(ClimberConstants.elevatorMotorPorts, ClimberConstants.inversionsElevator);
    encoder = elevatorMotors[0].getEncoder();
  }
   public void startClimb(){

   }
   public void endClimb(){

   }
   public double getPosition(){
     return this.position;
   }
   public double getVelocity(){
     return this.velocity;
   }
   public void grabSensors(){
     this.velocity = this.encoder.getVelocity();
     this.position = this.encoder.getPosition();
   }

  @Override
  public void periodic() {
    grabSensors();
  }
}