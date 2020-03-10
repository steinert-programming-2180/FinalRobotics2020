/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.drivemodes;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Constants.Units;
import frc.robot.subsystems.Drivetrain;

/**
 * An example command that uses an example subsystem.
 */
public class DefaultDrive extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

  Drivetrain drivetrain;
  Joystick leftStick, rightStick;
  double leftSpeed, rightSpeed;
  boolean reverseDrive = true;

  public DefaultDrive(Drivetrain driveSub, Joystick l, Joystick r) {
    // Use addRequirements() here to declare subsystem dependencies.
    drivetrain = driveSub;
    addRequirements(drivetrain);
    leftStick = l;
    rightStick = r;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //leftSpeed = DrivetrainConstants.defaultMotorFactor * ((5000/60) * DrivetrainConstants.gearRatio * DrivetrainConstants.wheelDiameter) * leftStick.getRawAxis(1);
    //rightSpeed = DrivetrainConstants.defaultMotorFactor * ((5000/60) * DrivetrainConstants.gearRatio * DrivetrainConstants.wheelDiameter) * rightStick.getRawAxis(1);
    leftSpeed = leftStick.getRawAxis(1) * -1 * DrivetrainConstants.defaultMotorFactor;
    rightSpeed = rightStick.getRawAxis(1) * -1 * DrivetrainConstants.defaultMotorFactor;
    //drivetrain.setDrive(leftSpeed, rightSpeed, Units.METERS);
    reverseDrive = !leftStick.getRawButton(3);
    drivetrain.setDrive(leftSpeed, rightSpeed, reverseDrive, Units.PERCENT);
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
