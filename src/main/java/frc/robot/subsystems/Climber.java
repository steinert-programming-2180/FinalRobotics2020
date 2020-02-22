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
  
  public Climber() {
    leftMotors = RobotUtilities.SetUpMotors(ClimberConstants.leftClimberMotorPorts);
    rightMotors = RobotUtilities.SetUpMotors(ClimberConstants.rightClimberMotorPorts);
    encoder = new CANEncoder(leftMotors[0]);

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
     
     return 0;
   }
   public double getVelocity(){
     return 0; 
   }
   public void grabSensors(){
     
   }

  @Override
  public void periodic() {

    // This method will be called once per scheduler run
  }
}
