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
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class runBalls extends CommandBase {
  /**
   * Creates a new runBalls.
   */
  Intake intake;
  long setTime;
  CANSparkMax conveyer;
  Shooter shooter;
  public runBalls(Shooter ks) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(ks);
    conveyer = new CANSparkMax(8, MotorType.kBrushless);
    shooter = ks;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    setTime = System.currentTimeMillis();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    conveyer.set(0.7);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    conveyer.set(0.0);
    shooter.stopShooting();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return System.currentTimeMillis() - setTime > 4000;
  }
}
