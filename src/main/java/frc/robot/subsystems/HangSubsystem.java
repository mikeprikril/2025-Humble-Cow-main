// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class HangSubsystem extends SubsystemBase {
  /** Creates a new HangSubsystem. */

  SparkMax hangMotor;

  SparkMaxConfig hangMotorConfig = new SparkMaxConfig();

  public HangSubsystem() {
    hangMotorConfig.inverted(false) //dont invert hang motor
    .idleMode(IdleMode.kBrake); //keep brake on

    hangMotor = new SparkMax(Constants.DrivebaseConstants.HangMotorID, MotorType.kBrushless);
    hangMotor.configure(hangMotorConfig, null, null);

  }

  public void HangRobot(){
  hangMotor.set(Constants.DrivebaseConstants.HangSpeed);
}

  public void UnwindHanger(){
  hangMotor.set(Constants.DrivebaseConstants.UnwindHangSpeed);
}

public void StopHangMotor(){
  hangMotor.stopMotor();
}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
