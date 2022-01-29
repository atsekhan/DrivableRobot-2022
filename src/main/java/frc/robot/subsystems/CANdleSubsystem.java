// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdle.*;
import com.ctre.phoenix.led.*;
import com.ctre.phoenix.led.ColorFlowAnimation.Direction;
import com.ctre.phoenix.led.LarsonAnimation.BounceMode;
import com.ctre.phoenix.led.TwinkleAnimation.TwinklePercent;
import com.ctre.phoenix.led.TwinkleOffAnimation.TwinkleOffPercent;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class CANdleSubsystem extends SubsystemBase {

  private CANdle m_candle;
  private final int LedCount = 8;
  private Animation m_toAnimate = null;

  public enum AnimationTypes {
    ColorFlow,
    Fire,
    Larson,
    Rainbow,
    RgbFade,
    SingleFade,
    Strobe,
    Twinkle,
    TwinkleOff,
    SetAll
}
/** Creates a new CANdleSubsystem. */
  public CANdleSubsystem() {

    if (Constants.RobotProperties.isCANdle) {
      // m_candle = new CANdle(Constants.CANdleConstants.CANdlePort, "FastFD");
      m_candle = new CANdle(Constants.CANdleConstants.CANdlePort);
      

      changeAnimation(AnimationTypes.SetAll);
      CANdleConfiguration configAll = new CANdleConfiguration();
      configAll.statusLedOffWhenActive = false;
      configAll.disableWhenLOS = false;
      configAll.stripType = LEDStripType.GRB;
      configAll.brightnessScalar = 0.1;
      configAll.vBatOutputMode = VBatOutputMode.Modulated;
      m_candle.configAllSettings(configAll, 100);

      // m_candle.setLEDs(50, 60, 70, 80, 0, 2);

    }
  }

  private void changeAnimation(AnimationTypes setall) {
  }


//tasks: Create 3 methods: LED=green, LED=blue, LED off

public void setLEDGreen()
{
  m_candle.setLEDs(10,200,10);
}

public void setLEDBlue()
{
  m_candle.setLEDs(10,10,200);
}

public void setLEDOff()
{
  m_candle.setLEDs(0,0,0);
}
}
