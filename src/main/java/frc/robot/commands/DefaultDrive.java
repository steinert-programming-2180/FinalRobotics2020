/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.subsystems.Drivetrain;

/**
 * An example command that uses an example subsystem.
 */
public class DefaultDrive extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

  Drivetrain sub;
  Joystick leftStick, rightStick;

  public DefaultDrive(Drivetrain driveSub) {
    // Use addRequirements() here to declare subsystem dependencies.
    sub = driveSub;
    addRequirements(sub);
    leftStick = new Joystick(0);
    rightStick = new Joystick(1);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double leftSpeed = -1 * DrivetrainConstants.defaultMotorFactor * leftStick.getRawAxis(1) * 2000;
    double rightSpeed = -1 * DrivetrainConstants.defaultMotorFactor * rightStick.getRawAxis(1) * 2000;
    SmartDashboard.putNumber("leftJoy", leftSpeed);
    SmartDashboard.putNumber("rightJoy", rightSpeed);
    this.sub.setDrive(leftSpeed, rightSpeed);
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
