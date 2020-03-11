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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.*;
import frc.robot.UniversalVar;
import frc.robot.commands.*;
import static frc.robot.Constants.ConveyerConstants;
import static frc.robot.RobotUtilities.*;
import static frc.robot.Constants.Units;

public class Conveyer extends SubsystemBase {
  /**
   * Creates a new Conveyer.
   */
  private CANSparkMax[] conveyerMotors;
  private CANEncoder conveyerEncoder;
  private DigitalInput topBeam, bottomBeam;

  private UniversalVar uniVar;

  private boolean topVal, bottomVal;
  private double conveyerVelocity, conveyerPosition;
  public Conveyer(DigitalInput kt, DigitalInput kb, UniversalVar uniVar) {
    this.uniVar = uniVar;
    conveyerMotors = SetUpMotors(ConveyerConstants.motorPorts, ConveyerConstants.motorInversions);
    topBeam = kt;
    bottomBeam = kb;

    conveyerEncoder = conveyerMotors[0].getEncoder();
  }

  public void setMotors(double speed) {
    conveyerMotors[0].set(speed);
  } public void bringUp () {
    this.setMotors(ConveyerConstants.defaultSpeed);
  } public void letDown () {
    this.setMotors(ConveyerConstants.defaultSpeed * -1);
  } public void stopSuck () {
    this.setMotors(0);
  }

  public boolean getBottomConveyerBeam () {
    return this.bottomVal;
  } public boolean getTopConveyerBeam () {
    return this.topVal;
  } public double getConveyerPosition () {
    return this.conveyerPosition;
  } public double getConveyerVelocity () {
    return this.conveyerVelocity;
  }

  public void grabSensors() {
    this.topVal = topBeam.get();
    SmartDashboard.putBoolean("IsRoom", topBeam.get());
    SmartDashboard.putBoolean("Bottom Beam", !bottomBeam.get());
    this.bottomVal = bottomBeam.get();
    this.conveyerPosition = conveyerEncoder.getPosition();
    this.conveyerVelocity = conveyerEncoder.getVelocity();
    
    uniVar.add("Conveyer-TopBeam", topVal);
    uniVar.add("Conveyer-BottomBeam", bottomVal);
    uniVar.add("Conveyer-Position", conveyerPosition);
    uniVar.add("Conveyer-Velocity", conveyerVelocity);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    grabSensors();
  }
}