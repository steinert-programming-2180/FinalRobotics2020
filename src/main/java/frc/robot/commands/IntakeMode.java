/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;

public class IntakeMode extends CommandBase {
  /**
   * Creates a new IntakeMode.
   */
  Intake intake;
  public IntakeMode(Intake, kintake) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(kintake);
    intake = kintake;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    intake.intakeDown();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intake.spinIntake();
    intake.runConveyer();
    intake.runFunnel();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.stopConveyer();
    intake.stopFunnel();
    intake.intakeUp();
    intake.stopIntake();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
