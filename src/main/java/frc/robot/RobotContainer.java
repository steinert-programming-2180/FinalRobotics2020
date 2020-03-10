/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
//import edu.wpi.first.wpilibj.XboxController.Button;
import frc.robot.Constants.IOPorts;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.BeamTripTrig;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  private DefaultDrive driveCom;
  private ShooterTest shooterCom;
  private BringBallUp raiseIntake;
  private RotateToColor colorRot;
  private RotateNumOfTimes wheelRot;

  private Drivetrain drivetrain;
  private Shooter shooter;
  private Intake intake;
  private Paddy paddy;

  DigitalInput beamTrip;

  Joystick left1 = Constants.customController.left1;
  Joystick right1 = Constants.customController.right1;
  Joystick operator1 = Constants.customController.operator1;

  JoystickButton shooterBtn = new JoystickButton(operator1, 1);
  JoystickButton conveyerBtn = new JoystickButton(operator1, 3);
  JoystickButton intakeRunButton = new JoystickButton(operator1, 2);
  JoystickButton clearShooter = new JoystickButton(operator1, 11);

  JoystickButton fullSpeed1 = new JoystickButton(left1, 1);
  JoystickButton fullSpeed2 = new JoystickButton(right1, 1);

  // BeamTripTrig funnelTrip = new BeamTripTrig(Constants.IOPorts.beamSensors[0]);
  // BeamTripTrig topTrip = new BeamTripTrig(Constants.IOPorts.beamSensors[5]);

  public RobotContainer() {
    //this.drivetrain.setDefaultCommand(new DefaultDrive(drivetrain));
    setUpSubsystems();
    configureButtonBindings();

    drivetrain.setDefaultCommand(new DefaultDrive(drivetrain, left1, right1));
  }

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // funnelTrip.and(topTrip.negate().and(b.negate())).whileActiveOnce(new BringBallUp(intake));
    // funnelTrip.and(topTrip.and(b.negate()).whenActive(new StopFunnel(intake)));
    // b.whenHeld(new FeedBallsToShooter(intake));
    shooterBtn.whenHeld(shooterCom);
    intakeBtn.whenHeld(intakeCom);
    conveyerBtn.whenHeld(conveyerCom);

    fullSpeed1.and(fullSpeed2).whenActive(new FullDrive);
  }

  private void setUpSubsystems() {
    drivetrain = new Drivetrain();
    shooter = new Shooter();
    intake = new Intake();
    //paddy = new Paddy();

    shooterCom = new ShooterTest(shooter, right1);
    intakeCom = new IntakeMode(intake);
    conveyerCom = new BringBallUp(intake);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return new PlaceholderAutonomousPlsDelete(); //This is why that command exists.  It stops the error without deleting the method.
  }
}
