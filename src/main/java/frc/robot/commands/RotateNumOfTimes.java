/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Paddy;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class RotateNumOfTimes extends CommandBase {
  Color currentColor;
  Color previousColor;
  Color startingColor;
  Paddy paddy;
  CANSparkMax wheelMotor;
  int counter;

  /**
   * Creates a new RotateNumOfTimes.
   */
  public RotateNumOfTimes(Paddy padSub) {
    addRequirements(padSub);
    paddy = padSub;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startingColor = this.paddy.getColor();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putNumber("Hi", System.currentTimeMillis());
    paddy.testColor();
    paddy.rotateWheel();
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
