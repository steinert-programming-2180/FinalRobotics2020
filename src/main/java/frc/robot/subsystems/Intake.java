/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

//import com.ctre.phoenix.motorcontrol.ControlMode;
//import com.ctre.phoenix.motorcontrol.FollowerType;
//import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.RobotUtilities.*;
import static frc.robot.Constants.IntakeConstants;
import static frc.robot.Constants.Units;

public class Intake extends SubsystemBase {
  /**
   * Creates a new Intake.
   */
  private DigitalInput[] beamTrips;
  private DigitalInput bottomTrip;
  private CANSparkMax[] conveyerMotors, funnelMotors;
  //private TalonSRX[] intakeMotors = new TalonSRX[2];
  private CANEncoder conveyerEncoder, funnelEncoder;
  private DoubleSolenoid leftIntakePiston, rightIntakePiston;

  private int ballsInStorage = 0;
  private double conveyerSpeed, conveyerPosition, funnelSpeed, funnelPosition;
  private boolean bottomTripVal;

  public Intake() {
    bottomTrip = new DigitalInput(IntakeConstants.beamTripPorts[2]);

    conveyerMotors = SetUpMotors(IntakeConstants.Conveyer.motorPorts, IntakeConstants.Conveyer.motorInversions);
    conveyerEncoder = conveyerMotors[0].getEncoder();
    conveyerEncoder.setPositionConversionFactor(IntakeConstants.Conveyer.positionConversionFactor);
    conveyerEncoder.setVelocityConversionFactor(IntakeConstants.Conveyer.velocityConversionFactor);

    funnelMotors = SetUpMotors(IntakeConstants.Funnel.motorPorts, IntakeConstants.Funnel.motorInversions);
    funnelEncoder = funnelMotors[0].getEncoder();
    funnelEncoder.setPositionConversionFactor(IntakeConstants.Funnel.positionConversionFactor);
    funnelEncoder.setVelocityConversionFactor(IntakeConstants.Funnel.velocityConversionFactor);

    //intakeMotors[0] = new TalonSRX(IntakeConstants.Intake.motorPorts[0]);
    //intakeMotors[0].setInverted(IntakeConstants.Intake.motorInversions[0]);
    //intakeMotors[1] = new TalonSRX(IntakeConstants.Intake.motorPorts[1]);
    //intakeMotors[1].follow(intakeMotors[1]);
    //intakeMotors[1].setInverted(IntakeConstants.Intake.motorInversions[0] ^ IntakeConstants.Intake.motorInversions[1]);

    leftIntakePiston = new DoubleSolenoid(IntakeConstants.Intake.solinoidPistonPorts[0][0], //Forward port
                                          IntakeConstants.Intake.solinoidPistonPorts[0][1]);  //Reverse port
    rightIntakePiston = new DoubleSolenoid(IntakeConstants.Intake.solinoidPistonPorts[1][0], //Forward port
                                          IntakeConstants.Intake.solinoidPistonPorts[1][1]); //Reverse port
  }

  public void runConveyer (double speed, Units liftingUnits){ //Runs in inches per second
    switch (liftingUnits) {
      case METERS: 
        speed = speed * 39.3701;
        break;
      case PERCENT:
        speed = speed * IntakeConstants.Conveyer.maxSpeed;
    }
    conveyerMotors[0].set(1);
  } public void stopConveyer () {
    runConveyer(0, Units.PERCENT);
  }

  public void intakeDown() {
    leftIntakePiston.set(Value.kForward);
    rightIntakePiston.set(Value.kForward);
  } public void intakeUp() {
    leftIntakePiston.set(Value.kReverse);
    rightIntakePiston.set(Value.kForward);
  } public void spinIntake () {
    //intakeMotors[0].set(ControlMode.PercentOutput, 1);
  }

  public void runFunnel (double speed, Units rotationUnits) {
    funnelMotors[0].set(speed);
  } public void stopFunnel () {
    runFunnel(0, Units.PERCENT);
  }

  public boolean getBottomTrip () {
    return this.bottomTripVal;
  }

  public void grabSensors() {
    this.bottomTripVal = bottomTrip.get();

    this.conveyerPosition = conveyerEncoder.getPosition();
    this.conveyerSpeed = conveyerEncoder.getVelocity();
    this.funnelPosition = funnelEncoder.getPosition();
    this.funnelSpeed = funnelEncoder.getVelocity();
  }

  @Override
  public void periodic() {
    grabSensors();
  }
}
