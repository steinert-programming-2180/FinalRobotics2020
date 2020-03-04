 /*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.PaddyConstants;
import static frc.robot.RobotUtilities.*;

import com.revrobotics.ColorSensorV3;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorMatch;
import  com.revrobotics.CANEncoder;


public class Paddy extends SubsystemBase {
  /**
   * Creates a new ExampleSubsystem.
   * */
  CANSparkMax[] turner;
  CANEncoder turnerEncoder;
  ColorSensorV3 colorSensor;
  ColorMatch colorMatcher;
  Color[] colors = new Color[4];
  Color currentColor;
  double motorPosition, motorSpeed;

  public Paddy() {
    turner = SetUpMotors(PaddyConstants.turnerMotors, PaddyConstants.inversionsTurner);
    turnerEncoder = turner[0].getEncoder();

    colorSensor = new ColorSensorV3(I2C.Port.kOnboard);
    colorMatcher = new ColorMatch();
    colors[0] = ColorMatch.makeColor(PaddyConstants.blueVals[0], PaddyConstants.blueVals[1], PaddyConstants.blueVals[2]); //Blue
    colors[1] = ColorMatch.makeColor(PaddyConstants.greenVals[0], PaddyConstants.greenVals[1], PaddyConstants.greenVals[2]); //Green
    colors[2] = ColorMatch.makeColor(PaddyConstants.redVals[0], PaddyConstants.redVals[1], PaddyConstants.redVals[2]); //Red
    colors[3] = ColorMatch.makeColor(PaddyConstants.yellowVals[0], PaddyConstants.yellowVals[1], PaddyConstants.yellowVals[2]); //Yellow
    for(Color i : colors){
        colorMatcher.addColorMatch(i);
    }
  }

  public void rotateWheel(){
    turner[0].set(PaddyConstants.defaultTurnSpeed);
  } public void stopWheel() {
    turner[0].set(0);
  }

  public Color getColor(){
    Color closestMatch = colorMatcher.matchClosestColor(colorSensor.getColor()).color;
    for(Color i : colors){
        if(closestMatch == i){
            return i;
        }
    }
    return null;
  }
  public void grabSensors() {
  }


  @Override
  public void periodic() {
    grabSensors();
    SmartDashboard.putNumber("R", this.currentColor.red);
    SmartDashboard.putNumber("G", this.currentColor.green);
    SmartDashboard.putNumber("B", this.currentColor.blue);
  }
}
