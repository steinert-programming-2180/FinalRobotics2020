 /*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import static frc.robot.Constants.PaddyConstants;
import static frc.robot.RobotUtilities.*;

import com.revrobotics.ColorSensorV3;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorMatch;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax.IdleMode;


public class Paddy extends SubsystemBase {
  /**
   * Creates a new ExampleSubsystem.
   * */
  CANSparkMax[] turner;
  CANEncoder turnerEncoder;
  ColorSensorV3 colorSensor;
  ColorMatch colorMatcher;

  Color blue;
  Color red;
  Color green;
  Color yellow;
  Color[] colors = new Color[4];
  Color targetColor;

  Color startColor;
  Color previousColor = null;
  Color currentColor = null;
  int semiRotations = 0;

  double motorPosition, motorSpeed;

  long startTime;

  public Paddy() {
    turner = SetUpMotors(PaddyConstants.turnerMotors, PaddyConstants.inversionsTurner);
    turner[0].setIdleMode(IdleMode.kBrake);
    turnerEncoder = turner[0].getEncoder();

    colorSensor = new ColorSensorV3(Port.kOnboard);
    colorMatcher = new ColorMatch();
    targetColor = PaddyConstants.targetColor;

    blue = PaddyConstants.blue;
    green = PaddyConstants.green;
    red = PaddyConstants.red;
    yellow = PaddyConstants.yellow;

    colors[0] = blue;
    colors[1] = green;
    colors[2] = red;
    colors[3] = yellow;

    for(Color i : colors){
      colorMatcher.addColorMatch(i);
    }
  }

  public void getTarget(){
    String data = DriverStation.getInstance().getGameSpecificMessage().toUpperCase();

    //We set the targetColor to a different color to account for the offset on the wheel.

    switch(data.charAt(0)){
      case 'R':
        Constants.PaddyConstants.targetColor = blue;
        break;
      case 'B':
        Constants.PaddyConstants.targetColor = red;
        break;
      case 'G':
        Constants.PaddyConstants.targetColor = yellow;
        break;
      case 'Y':
        Constants.PaddyConstants.targetColor = green;
        break;
      default:
        Constants.PaddyConstants.targetColor = null;
        break;
    }
    targetColor = Constants.PaddyConstants.targetColor;
  }

  public void rotateWheel(){
    //stop after seing the color 8 times
    startColor = getColor();
    rotateWheelHelper(startColor);
  }

  public void rotateWheelHelper(Color start){
    if(previousColor == null){
      previousColor = start;
    }
    currentColor = getColor();
    if(previousColor != currentColor && previousColor == start){
      semiRotations += 1;
    }

    if(semiRotations < 9){
      turner[0].set(PaddyConstants.defaultTurnSpeed);
      previousColor = currentColor;
      currentColor = getColor();
      rotateWheelHelper(start);
    }
  }

  public void turnToColor(){
    if(targetColor == null){
      return;
    }

    while(getColor() != targetColor){
      turner[0].set(PaddyConstants.defaultTurnSpeed);
    }
    turner[0].set(0);
  }

  public void stopWheel() {
    turner[0].set(0);
  }

  public Color getColor(){
    Color closestMatch = colorMatcher.matchClosestColor(colorSensor.getColor()).color;
    return closestMatch;
  }

  public void grabSensors() {
    SmartDashboard.putNumber("RedVal", colorSensor.getColor().red);
  }


  @Override
  public void periodic() {
    grabSensors();
    // SmartDashboard.putNumber("R", this.currentColor.red);
    // SmartDashboard.putNumber("G", this.currentColor.green);
    // SmartDashboard.putNumber("B", this.currentColor.blue);
  }
}
