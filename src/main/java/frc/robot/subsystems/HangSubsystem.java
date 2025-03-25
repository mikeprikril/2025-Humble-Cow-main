// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
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

  RelativeEncoder hangEncoder;

  SparkMaxConfig hangMotorConfig = new SparkMaxConfig();

  public HangSubsystem() {
    hangMotorConfig.inverted(false) //dont invert hang motor
    .idleMode(IdleMode.kBrake); //keep brake on

    hangMotor = new SparkMax(Constants.DrivebaseConstants.HangMotorID, MotorType.kBrushless);
    hangMotor.configure(hangMotorConfig, null, null);

    hangEncoder = hangMotor.getEncoder();

    AndrewMotor = new VictorSP(4);

    rotatorServo = new Servo(2);
    rotatorServo.setBoundsMicroseconds(2000, 1800, 1500, 1200, 0);
  }

  public void HangRobot(){
  if (hangEncoder.getPosition() > Constants.HangConstants.RobotHanging){
  hangMotor.set(Constants.DrivebaseConstants.HangSpeed);
  AndrewMotor.set(1);
  }
  else {
    hangMotor.stopMotor();
    AndrewMotor.stopMotor();
  }
}

public void HangerVertical(){
  if (hangEncoder.getPosition() > 0){
  hangMotor.set(Constants.DrivebaseConstants.HangSpeed*.6);
  AndrewMotor.set(1);
  }
  else {
    hangMotor.stopMotor();
    AndrewMotor.stopMotor();
  }
}

  public void ExtendHanger(){
    if (hangEncoder.getPosition() < Constants.HangConstants.HangerHorizontal){
  hangMotor.set(Constants.DrivebaseConstants.UnwindHangSpeed*.6);
  AndrewMotor.stopMotor();
    }
  else {
    hangMotor.stopMotor();
    AndrewMotor.stopMotor();
  }
}

public void HangerJustABitOut(){
  if (hangEncoder.getPosition() < Constants.HangConstants.HangerJustABitOut){
hangMotor.set(Constants.DrivebaseConstants.UnwindHangSpeed*.6);
AndrewMotor.stopMotor();
  }
else {
  hangMotor.stopMotor();
  AndrewMotor.stopMotor();
  }
}

public double GetHangEncoderPosition(){
  return hangEncoder.getPosition();
}


public void StopHangMotor(){
  hangMotor.stopMotor();
  AndrewMotor.stopMotor();
}

public void RotateHangBar(double ServoPosition){
rotatorServo.setPosition(ServoPosition);
}

public void ResetHangEncoder(){

    hangMotor.getEncoder().setPosition(0); //reset encoder 
  }



  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Servo Position", rotatorServo.getPulseTimeMicroseconds());
    SmartDashboard.putNumber("Hang Encoder Value", hangEncoder.getPosition());
  }
}
