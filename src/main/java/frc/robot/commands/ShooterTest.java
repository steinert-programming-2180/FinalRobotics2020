/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.Units;
import frc.robot.subsystems.Shooter;

public class ShooterTest extends CommandBase {
  /**
   * Creates a new ShooterTest.
   */
  Shooter shooter;
  Joystick stick;
  public ShooterTest(Shooter kShooter, Joystick kStick) {
    addRequirements(kShooter);
    shooter = kShooter;
    stick = kStick;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shooter.shootBall((stick.getRawAxis(2) - 1) * -0.5, Units.PERCENT);
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