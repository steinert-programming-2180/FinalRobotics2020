/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FollowerType;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.RobotUtilities.*;
import static frc.robot.Constants.IntakeConstants;
import static frc.robot.Constants.Units;

public class Intake extends SubsystemBase {
  /**
   * Creates a new Intake.
   */
  private DigitalInput[] beamTrips;
  private CANSparkMax[] conveyerMotors, funnelMotors;
  private TalonSRX[] intakeMotors = new TalonSRX[2];
  private CANEncoder conveyerEncoder, funnelEncoder;

  private int ballsInStorage = 0;

  public Intake() {
    conveyerMotors = SetUpMotors(IntakeConstants.Conveyer.motorPorts, IntakeConstants.Conveyer.motorInversions);
    conveyerEncoder = conveyerMotors[0].getEncoder();
    conveyerEncoder.setPositionConversionFactor(IntakeConstants.Conveyer.positionConversionFactor);
    conveyerEncoder.setVelocityConversionFactor(IntakeConstants.Conveyer.velocityConversionFactor);

    funnelMotors = SetUpMotors(IntakeConstants.Funnel.motorPorts, IntakeConstants.Funnel.motorInversions);
    funnelEncoder = funnelMotors[0].getEncoder();
    funnelEncoder.setPositionConversionFactor(IntakeConstants.Funnel.positionConversionFactor);
    funnelEncoder.setVelocityConversionFactor(IntakeConstants.Funnel.velocityConversionFactor);

    intakeMotors[0] = new TalonSRX(IntakeConstants.Intake.motorPorts[0]);
    intakeMotors[0].setInverted(IntakeConstants.Intake.motorInversions[0]);
    intakeMotors[1] = new TalonSRX(IntakeConstants.Intake.motorPorts[1]);
    intakeMotors[1].follow(intakeMotors[1]);
    intakeMotors[1].setInverted(IntakeConstants.Intake.motorInversions[0] ^ IntakeConstants.Intake.motorInversions[1]);
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
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
