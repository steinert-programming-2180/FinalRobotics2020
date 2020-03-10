 /*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
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
  private final I2C.Port i2cPort = I2C.Port.kOnboard;

  public Paddy() {
    turner = SetUpMotors(PaddyConstants.turnerMotors, PaddyConstants.inversionsTurner);
    turnerEncoder = turner[0].getEncoder();

    colorSensor = new ColorSensorV3(i2cPort);
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

  public void testColor(){
    if (getColor() == blue) {
      SmartDashboard.putString("Color", "Blue");
    } else if (getColor() == red) {
      SmartDashboard.putString("Color", "REd");
    } else if (getColor() == green) {
      SmartDashboard.putString("Color", "Green");
    } else if (getColor() == yellow) {
      SmartDashboard.putString("Color", "Yellow");
    } else {
      SmartDashboard.putString("Color", "Unknown");
    }
  }

  public void turnToColor(){
    if(targetColor == null){
      return;
    }

    while(getColor() != targetColor){
      turner[0].set(PaddyConstants.defaultTurnSpeed);
    }
  }

  public void stopWheel() {
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
