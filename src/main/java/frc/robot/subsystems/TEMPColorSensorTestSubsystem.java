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

public class TEMPColorSensorTestSubsystem extends SubsystemBase {

  private final I2C.Port i2cPort = I2C.Port.kOnboard;
  private ColorSensorV3 colorSensor;
  private Color lastSeenColor;

  // Color match test
  private final ColorMatch m_colorMatcher = new ColorMatch();
  
  /** Creates a new TEMPColorSensorTestSubsystem. */
  public TEMPColorSensorTestSubsystem() {
    if (RobotProperties.isColorSensor) {
      colorSensor = new ColorSensorV3(i2cPort);
      if (! colorSensor.isConnected()) {  // cannot determine the presence of the sensor, so disable it
        RobotProperties.isColorSensor =  false;
      }

      m_colorMatcher.addColorMatch(Color.kBlue);
      m_colorMatcher.addColorMatch(Color.kGreen);
      m_colorMatcher.addColorMatch(Color.kRed);
      m_colorMatcher.addColorMatch(Color.kYellow);

    }
  }

  public String getSeenColor() {   // It will return most closely matched color as ENUM
    // if (colorSensor.getColor() != null)
      lastSeenColor = colorSensor.getColor();
      ColorMatchResult match = m_colorMatcher.matchClosestColor(lastSeenColor);
      String colorString ;
      if (match.color == Color.kBlue) {
        colorString = "Blue";
      } else if (match.color == Color.kRed) {
        colorString = "Red";
      } else if (match.color == Color.kGreen) {
        colorString = "Green";
      } else if (match.color == Color.kYellow) {
        colorString = "Yellow";
      } else {
        colorString = "Unknown";
      }
    return colorString + " Confidence " + match.confidence;
  }

  public int getObjectProximity() {   // It will return most closely matched color as ENUM
    return colorSensor.getProximity();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
