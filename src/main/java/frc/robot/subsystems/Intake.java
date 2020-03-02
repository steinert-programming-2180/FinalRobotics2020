/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotUtilities;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.Units;

public class Intake extends SubsystemBase {
  /**
   * Creates a new Intake.
   */
  private DigitalInput[] beamTrips;
  private CANSparkMax motor;
  int ballsInStorage = 0;
  public Intake() {
    beamTrips = new DigitalInput[IntakeConstants.beamTripPorts.length];
    motor = new CANSparkMax(IntakeConstants.conveyorMotorPort, MotorType.kBrushless);
    for(int i = 0; i < IntakeConstants.beamTripPorts.length; i++){
      beamTrips[i] = new DigitalInput(IntakeConstants.beamTripPorts[i]);
    }
  }

  void turnMotor(Units liftingUnits){
    motor.set(1);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
