// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ChuteSubsystem extends SubsystemBase {
  /** Creates a new ChuteSubsystem. */
  Servo chuteServo;
  
  public ChuteSubsystem() {
    chuteServo = new Servo(1);
  }

  public void ServoOpenChute(){
    chuteServo.set(1);
  }

  public void ServoCloseChute(){
    chuteServo.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
