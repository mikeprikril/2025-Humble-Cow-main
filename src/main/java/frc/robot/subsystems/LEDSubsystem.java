// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LEDSubsystem extends SubsystemBase {
  /** Creates a new LEDSubsystem. */
  AddressableLED adLED;
  AddressableLEDBuffer adLEDBuffer;

  public LEDSubsystem() {
    adLED = new AddressableLED(Constants.DrivebaseConstants.LEDPWMPort);
    adLEDBuffer = new AddressableLEDBuffer(64);
    adLED.setLength(adLEDBuffer.getLength());
    adLED.setData(adLEDBuffer);
    adLED.start();
  }

  public void TargetLight (boolean canRobotSeeTag)
  {
    if(canRobotSeeTag == true)
    {
      for (int i = 0; i < adLEDBuffer.getLength(); i++)
      {
        adLEDBuffer.setRGB(i, 150, 50, 20); //green is 0, 192, 0
      }
    }

      else
    {
      for (int i = 0; i < adLEDBuffer.getLength(); i++)
      {
        adLEDBuffer.setRGB(i, 0, 0, 0);
      }
    }
    adLED.setData(adLEDBuffer);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
