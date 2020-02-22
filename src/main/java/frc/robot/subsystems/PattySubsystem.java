/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.PaddyConstants;

import com.revrobotics.ColorSensorV3;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorMatch;

public class PattySubsystem extends SubsystemBase {
  /**
   * Creates a new ExampleSubsystem.
   * */

  CANSparkMax turner;
  ColorSensorV3 colorSensor;
  ColorMatch colorMatcher;

  private final Color kBlueTarget = ColorMatch.makeColor(0.232154, 0.427, 0.429);
  private final Color kGreenTarget = ColorMatch.makeColor(0.197, 0.561, 0.240);
  private final Color kRedTarget = ColorMatch.makeColor(0.291504, 0.322754, 0.110107);
  private final Color kYellowTarget = ColorMatch.makeColor(0.322266, 0.571777, 0.105957);
  private final Color target = new Color(0, 0, 0);
  Color[] colors = {kBlueTarget, kGreenTarget, kRedTarget, kYellowTarget};

  public PattySubsystem() {
    turner = new CANSparkMax(PaddyConstants.pattyMotor, MotorType.kBrushless);
    colorSensor = new ColorSensorV3(I2C.Port.kOnboard);
    colorMatcher = new ColorMatch();
    
    for(Color i : colors){
        colorMatcher.addColorMatch(i);
    }
  }

  public void rotateWheel(){
      turner.set(1);
  }

  public Color getColor(){
    Color closestMatch = colorMatcher.matchClosestColor(colorSensor.getColor()).color;
    for(Color i : colors){
        if(closestMatch == i){
            return i;
        }
    }
    return Color.kWhite;
  }

  public void setColor(){
      
  }

  public void checkColor(){

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
