// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class HangSubsystem extends SubsystemBase {
  /** Creates a new HangSubsystem. */

  SparkMax hangMotor;
  VictorSP AndrewMotor;
  Servo rotatorServo;

  SparkMaxConfig hangMotorConfig = new SparkMaxConfig();

  public HangSubsystem() {
    hangMotorConfig.inverted(false) //dont invert hang motor
    .idleMode(IdleMode.kBrake); //keep brake on

    hangMotor = new SparkMax(Constants.DrivebaseConstants.HangMotorID, MotorType.kBrushless);
    hangMotor.configure(hangMotorConfig, null, null);

    AndrewMotor = new VictorSP(4);

    rotatorServo = new Servo(2);
    rotatorServo.setBoundsMicroseconds(2000, 1800, 1500, 1200, 0);
  }

  public void HangRobot(){
  hangMotor.set(Constants.DrivebaseConstants.HangSpeed);
  AndrewMotor.set(1);
}

  public void UnwindHanger(){
  hangMotor.set(Constants.DrivebaseConstants.UnwindHangSpeed);
  AndrewMotor.stopMotor();
}

public void StopHangMotor(){
  hangMotor.stopMotor();
  AndrewMotor.stopMotor();
}

public void RotateHangBar(double ServoPosition){
rotatorServo.setPosition(ServoPosition);
}


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Servo Position", rotatorServo.getPulseTimeMicroseconds());
  }
}
