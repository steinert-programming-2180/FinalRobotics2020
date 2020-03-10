/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.*;
import frc.robot.commands.*;
import static frc.robot.Constants.FunnelConstants;
import static frc.robot.RobotUtilities.*;
import static frc.robot.Constants.Units;

public class Funnel extends SubsystemBase {
  /**
   * Creates a new Funnel.
   */
  private CANSparkMax[] funnelMotors;
  private CANEncoder funnelEncoder;
  private DigitalInput beam;

  private boolean beamVal;
  private double funnelVelocity, funnelPosition;
  public Funnel(DigitalInput kbeam) {
    funnelMotors = SetUpMotors(FunnelConstants.motorPorts, FunnelConstants.motorInversions);
    beam = kbeam;

    funnelEncoder = funnelMotors[0].getEncoder();
  }

  public void setMotors(double speed) {
    funnelMotors[0].set(speed);
  } public void suckIn () {
    this.setMotors(FunnelConstants.defaultSpeed);
  } public void reverseSuck () {
    this.setMotors(FunnelConstants.defaultSpeed * -1);
  } public void stopSuck () {
    this.setMotors(0);
  }

  public boolean getFunnelBeam () {
    return this.beamVal;
  } public double getFunnelPosition () {
    return this.funnelPosition;
  } public double getFunnelVelocity () {
    return this.funnelVelocity;
  }

  public void grabSensors() {
    this.beamVal = beam.get();
    this.funnelPosition = funnelEncoder.getPosition();
    this.funnelVelocity = funnelEncoder.getVelocity();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    grabSensors();
  }
}
