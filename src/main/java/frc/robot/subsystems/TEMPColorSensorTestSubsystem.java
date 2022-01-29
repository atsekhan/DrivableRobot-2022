// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.ColorMatch;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.ColorMatchResult;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotProperties;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class TEMPColorSensorTestSubsystem extends SubsystemBase {

  private final I2C.Port i2cPort = I2C.Port.kOnboard;
  private ColorSensorV3 colorSensor;
  private Color lastSeenColor;

  // Color match test
  private final ColorMatch m_colorMatcher = new ColorMatch();
  private final Color kBlueTarget = new Color(0.143, 0.427, 0.429);
  private final Color kGreenTarget = new Color(0.197, 0.561, 0.240);
  private final Color kRedTarget = new Color(0.561, 0.232, 0.114);
  private final Color kYellowTarget = new Color(0.361, 0.524, 0.113);

  
  /** Creates a new TEMPColorSensorTestSubsystem. */
  public TEMPColorSensorTestSubsystem() {
    if (RobotProperties.isColorSensor) {
      colorSensor = new ColorSensorV3(i2cPort);
      if (! colorSensor.isConnected()) {  // cannot determine the presence of the sensor, so disable it
        RobotProperties.isColorSensor =  false;
      }

           
      m_colorMatcher.addColorMatch(kBlueTarget);
      m_colorMatcher.addColorMatch(kGreenTarget);
      m_colorMatcher.addColorMatch(kRedTarget);
      m_colorMatcher.addColorMatch(kYellowTarget);

    }
  }

  public String getSeenColor() {   // It will return most closely matched color as ENUM
    // if (colorSensor.getColor() != null)
      lastSeenColor = colorSensor.getColor();
      ColorMatchResult match = m_colorMatcher.matchClosestColor(lastSeenColor);
      String colorString;
      if (match.color == kBlueTarget) {
        colorString = "Blue";
      } else if (match.color == kRedTarget) {
        colorString = "Red";
      } else if (match.color == kGreenTarget) {
        colorString = "Green";
      } else if (match.color == kYellowTarget) {
        colorString = "Yellow";
      } else {
        colorString = "Unknown";
      }
      
    return colorString + " Confidence " + match.confidence + " RAW " + lastSeenColor;
  }

  public int getObjectProximity() {   // It will return most closely matched color as ENUM
    return colorSensor.getProximity();
  }

  public boolean isBallRed(){    // Will return if a ball is red
    lastSeenColor = colorSensor.getColor();
    ColorMatchResult match = m_colorMatcher.matchClosestColor(lastSeenColor);
    if (match.color == kRedTarget)
      return true;
    else
      return false;
  }

  public boolean isBallBlue(){     // Will return if a ball is blue
    lastSeenColor = colorSensor.getColor();
    ColorMatchResult match = m_colorMatcher.matchClosestColor(lastSeenColor);
    if (match.color == kBlueTarget)
      return true;
    else
      return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
