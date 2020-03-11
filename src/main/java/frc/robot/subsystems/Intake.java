/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FollowerType;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
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
  private CANEncoder conveyerEncoder, funnelEncoder;
  private DoubleSolenoid intakePiston;
  private TalonSRX leftIntake, rightIntake;

  private int ballsInStorage = 0;
  private double conveyerSpeed, conveyerPosition, funnelSpeed, funnelPosition;
  private boolean bottomTripVal;


  public Intake() {
    leftIntake = new TalonSRX(14);
    rightIntake = new TalonSRX(15);

    intakePiston = new DoubleSolenoid(0, 1);
  }

  public void intakeDown() {
    intakePiston.set(Value.kReverse);
  }
  public void intakeUp() {
    intakePiston.set(Value.kForward);
  }
  public void spinIntake () {
    leftIntake.set(ControlMode.PercentOutput, -1);
    rightIntake.set(ControlMode.PercentOutput, 1);
  }
  public void reverseIntake () {
    leftIntake.set(ControlMode.PercentOutput, 1);
    rightIntake.set(ControlMode.PercentOutput, -1);
  }
  public void stopIntake () {
    leftIntake.set(ControlMode.PercentOutput, 0);
    rightIntake.set(ControlMode.PercentOutput, 0);
  }

  public void grabSensors() {
    // this.conveyerPosition = conveyerEncoder.getPosition();
    // this.conveyerSpeed = conveyerEncoder.getVelocity();
    // this.funnelPosition = funnelEncoder.getPosition();
    // this.funnelSpeed = funnelEncoder.getVelocity();
  }

  @Override
  public void periodic() {
    grabSensors();
  }
}
