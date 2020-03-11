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
import frc.robot.RobotUtilities;

import static frc.robot.Constants.ClimberConstants;

public class Climber extends SubsystemBase {
  /**
   * Creates a new Climber.
   */
  CANSparkMax[] leftMotors = new CANSparkMax[ClimberConstants.leftClimberMotorPorts.length];
  CANSparkMax[] rightMotors = new CANSparkMax[ClimberConstants.rightClimberMotorPorts.length];
  CANEncoder encoder;
  private double velocity, position;
  
  public Climber() {
    leftMotors = RobotUtilities.SetUpMotors(ClimberConstants.leftClimberMotorPorts);
    rightMotors = RobotUtilities.SetUpMotors(ClimberConstants.rightClimberMotorPorts);
     encoder = new CANEncoder(rightMotors[0]);
     

    leftMotors[0].setInverted(true);
    for(int i = 1; i < leftMotors.length; i++){
      leftMotors[i].follow(leftMotors[0], false);
      rightMotors[0].setInverted(false);
    }
    for(int i = 1; i < rightMotors.length; i++){
      rightMotors[i].follow(rightMotors[0], false);
    }
  }
   public void startClimb(){

   }
   public void endClimb(){

   }
   public double getPosition(){
     return this.getPosition();
   }
   public double getVelocity(){
    return this.getVelocity();
   }
   public void grabSensors(){
     this.velocity = this.encoder.getVelocity();
     this.position = this.encoder.getPosition();
   }

  @Override
  public void periodic() {
   grabSensors();
    // This method will be called once per scheduler run
  }
}
