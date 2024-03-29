/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.simpledriveshoot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;


import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ConveyerConstants;
import frc.robot.Constants.Units;
import frc.robot.subsystems.*;

public class placeholder extends CommandBase {
  /**
   * Creates a new placeholder.
   */
  Drivetrain drivetrain;
  Shooter shooter;
  long setTime, time;
  CANSparkMax conveyer;
  public placeholder(Drivetrain kdrive, Shooter kshoot, long ktime) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(kdrive, kshoot);
    drivetrain = kdrive;
    shooter = kshoot;
    time = ktime;
    conveyer = new CANSparkMax(ConveyerConstants.motorPorts[0], MotorType.kBrushless);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    setTime = System.currentTimeMillis();
    
    shooter.shootBall();
    drivetrain.setDrive(-0.15, -0.15, Units.PERCENT);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    conveyer.set(0.7);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.setDrive(0,0,Units.PERCENT);
    shooter.stopShooting();
    conveyer.set(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return System.currentTimeMillis() - setTime > time * 1000;
  }
}
