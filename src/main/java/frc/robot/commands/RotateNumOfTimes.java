/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Paddy;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class RotateNumOfTimes extends CommandBase {
  Color currentColor;
  Color previousColor;
  Color startingColor;
  Paddy sub;
  CANSparkMax wheelMotor;
  int counter;
  Joystick joy1, joy2; 

  /**
   * Creates a new RotateNumOfTimes.
   */
  public RotateNumOfTimes(Paddy padSub) {
    sub = padSub;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    wheelMotor = new CANSparkMax(1, MotorType.kBrushless);
    joy1 = new Joystick(0);
    joy2 = new Joystick(1);
    startingColor = this.sub.getColor();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    currentColor = this.sub.getColor();

    if((currentColor != previousColor) && (previousColor == startingColor)){
      counter++;
    }
    if(counter > 6){
    wheelMotor.set(0);
   }else{
    wheelMotor.set(0.5);
   }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
