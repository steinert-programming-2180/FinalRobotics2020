/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.*;
import frc.robot.commands.*;
import frc.robot.commands.drivemodes.*;
import frc.robot.commands.simpledriveshoot.*;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.BeamTripTrig;
import frc.robot.Constants.Units;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  private Drivetrain drivetrain;
  private Shooter shooter;
  private Intake intake;
  private Paddy paddy;
  private Conveyer conveyer;
  private Funnel funnel;
  private Climber climb;

  UniversalVar uniVar;

  DigitalInput funnelBeam = new DigitalInput(5);
  DigitalInput bottomBeam = new DigitalInput(7);
  DigitalInput veryBottom = new DigitalInput(8);
  DigitalInput topBeam = new DigitalInput(6);

  Joystick left1 = new Joystick(0);
  Joystick right1 = new Joystick(1);
  Joystick operator1 = new Joystick(2);

  JoystickButton shooterBtn = new JoystickButton(operator1, 1);
  JoystickButton intakeRunButton = new JoystickButton(operator1, 2);
  JoystickButton clearIntake = new JoystickButton(operator1, 10);
  JoystickButton clearShooter = new JoystickButton(operator1, 9);
  JoystickButton clearConveyer = new JoystickButton(operator1, 8);
  JoystickButton runConveyer = new JoystickButton(operator1, 7);
  JoystickButton runPaddy = new JoystickButton(operator1, 5);

  JoystickButton fullSpeed1 = new JoystickButton(left1, 1);
  JoystickButton fullSpeed2 = new JoystickButton(right1, 1);

  BeamTripTrig funnelTrip = new BeamTripTrig(funnelBeam);
  BeamTripTrig bottomTrip = new BeamTripTrig(bottomBeam);
  BeamTripTrig topTrip = new BeamTripTrig(topBeam);

  // BeamTripTrig funnelTrip = new BeamTripTrig(Constants.IOPorts.beamSensors[0]);
  // BeamTripTrig topTrip = new BeamTripTrig(Constants.IOPorts.beamSensors[5]);

  public RobotContainer(UniversalVar uniVar) {
    this.uniVar = uniVar;
    //this.drivetrain.setDefaultCommand(new DefaultDrive(drivetrain));
    setUpSubsystems();
    configureButtonBindings();

    drivetrain.setDefaultCommand(new DefaultDrive(drivetrain, left1, right1));
  }

  private void configureButtonBindings() {
    // funnelTrip.and(topTrip.negate().and(b.negate())).whileActiveOnce(new BringBallUp(intake));
    // funnelTrip.and(topTrip.and(b.negate()).whenActive(new StopFunnel(intake)));
    // b.whenHeld(new FeedBallsToShooter(intake));
    final Trigger automaticMode = runConveyer.or(clearConveyer).negate();
    final Trigger roomToLoad = topTrip.negate();

    intakeRunButton.whenHeld(new IntakeMode(intake)); //This one isn't inlined because it isn't primitive

    shooterBtn.whileHeld(() -> shooter.shootBall(operator1.getRawAxis(2), Units.PERCENT))
              .whenReleased(() -> shooter.stopShooting());

    runConveyer.whenPressed(() -> conveyer.bringUp()).whenPressed(() -> funnel.suckIn())
              .whenReleased(() -> conveyer.stopSuck()).whenReleased(() -> funnel.stopSuck());

    clearIntake.whenPressed(() -> intake.reverseIntake())
                .whenReleased(() -> intake.stopIntake());

    clearShooter.whenPressed(() -> shooter.shootBall(-1, Units.PERCENT))
                .whenReleased(() -> shooter.stopShooting());

    clearConveyer.whenPressed(() -> conveyer.letDown()).whenPressed(() -> funnel.reverseSuck())
                  .whenReleased(() -> conveyer.stopSuck()).whenReleased(() -> funnel.stopSuck());
    
    runPaddy.whenPressed(() -> paddy.turnToColor());

    //These functions manage the automatic storage of balls.  The first runs the funnel, the second the conveyer
    // (funnelTrip.and(roomToLoad.negate())).and(automaticMode).negate().whileActiveContinuous(() -> funnel.suckIn());
    // (funnelTrip.and(roomToLoad.negate())).and(automaticMode).whileActiveContinuous(() -> funnel.stopSuck());

    // (roomToLoad.and(bottomTrip.negate().or(funnelTrip))).and(automaticMode).whileActiveContinuous(() -> conveyer.bringUp());
    // (roomToLoad.and(bottomTrip.negate().or(funnelTrip))).and(automaticMode).negate().whileActiveContinuous(() -> conveyer.stopSuck());

    //Simple speed control code
    fullSpeed1.and(fullSpeed2).whileActiveOnce(new FullDrive(drivetrain, left1, right1));
    fullSpeed1.or(fullSpeed2).whileActiveOnce(new SeventyFivePower(drivetrain, left1, right1));
  }

  private void setUpSubsystems() {
    drivetrain = new Drivetrain(uniVar);
    shooter = new Shooter(uniVar);
    intake = new Intake();
    funnel = new Funnel(funnelBeam);
    conveyer = new Conveyer(topBeam, bottomBeam, uniVar);
    climb = new Climber(uniVar);
    paddy = new Paddy();
  }
  
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return new SimpleDriveShoot(drivetrain, shooter, (long)6);
  }
}
