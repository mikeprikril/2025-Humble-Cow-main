// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsytem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ScoreCoral extends Command {
  /** Creates a new ScoreCoral. */
  public final ElevatorSubsystem elevator;
  public final ArmSubsytem arm;
  public final SwerveSubsystem swerve;
  public final Joystick panel;
  private final ChassisSpeeds scoreSpeeds;
  public final Timer timer;

  public ScoreCoral(ElevatorSubsystem m_elevator, ArmSubsytem m_arm, SwerveSubsystem m_swerve, Joystick m_panel) {
    // Use addRequirements() here to declare subsystem dependencies.
    elevator = m_elevator;
    arm = m_arm;
    swerve = m_swerve;
    panel = m_panel;
    
    scoreSpeeds = new ChassisSpeeds(0, 0, 0);
    timer = new Timer();

    addRequirements(elevator, arm, swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //make sure elevator is up

    //arm down slow
    if (arm.GetArmEncoderPosition() < Constants.ArmConstants.ScorePosition){
      arm.AutoArmMove(Constants.ArmConstants.ArmDownSpeed*.4);
      arm.StopGripper(); 
    }
    else if (arm.GetArmEncoderPosition() > Constants.ArmConstants.ScorePosition && arm.GetArmEncoderPosition() < Constants.ArmConstants.AlgaePosition){
      arm.AutoArmMove(Constants.ArmConstants.ArmDownSpeed*.3);
      arm.GripperSpitOut();
    }
    else {
    arm.StopArm();
    arm.GripperSpitOut();
    }

    // drive
    if (arm.GetArmEncoderPosition() < Constants.ArmConstants.ScorePosition) {
      scoreSpeeds.vxMetersPerSecond = 0; //don't move if arm isn't down yet
    }
    else scoreSpeeds.vxMetersPerSecond = Constants.ArmConstants.DriveBackSpeed; //move robot

    //send values to swervedrive
    scoreSpeeds.vyMetersPerSecond = 0; //don't move side to side
    scoreSpeeds.omegaRadiansPerSecond = 0; //don't spin

    swerve.drive(scoreSpeeds);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    timer.stop();

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return !panel.getRawButton(Constants.OperatorConstants.ScoreCoralButton);
  }
}
