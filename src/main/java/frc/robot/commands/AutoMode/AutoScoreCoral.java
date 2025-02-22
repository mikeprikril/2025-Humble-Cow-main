// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoMode;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsytem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AutoScoreCoral extends Command {
  /** Creates a new ScoreCoral. */
  public final ElevatorSubsystem elevator;
  public final ArmSubsytem arm;
  public final SwerveSubsystem swerve;
  private final ChassisSpeeds scoreSpeeds;
  public final Timer timer;

  public AutoScoreCoral(ElevatorSubsystem m_elevator, ArmSubsytem m_arm, SwerveSubsystem m_swerve) {
    // Use addRequirements() here to declare subsystem dependencies.
    elevator = m_elevator;
    arm = m_arm;
    swerve = m_swerve;
    
    scoreSpeeds = new ChassisSpeeds(0, 0, 0);
    timer = new Timer();

    addRequirements(elevator, arm, swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //make sure elevator is up
    if (elevator.GetElevatorEncoderPosition() < Constants.ElevatorConstants.UpLimit){ 
      elevator.AutoElevator(Constants.ElevatorConstants.AutoUpSpeed*.3);
    }
    else elevator.StopElevator();

    //arm down slow
    if (arm.GetArmEncoderPosition() < Constants.ArmConstants.ScorePosition){
      arm.AutoArmMove(Constants.ArmConstants.ArmDownSpeed*.3); 
    }
    else {
      arm.StopArm();
      timer.start();
    }

    //gripper and drive
    if (arm.GetArmEncoderPosition() > Constants.ArmConstants.ScorePosition && timer.get() < Constants.ArmConstants.WaitScore){
          arm.GripperSpitOut();
          scoreSpeeds.vxMetersPerSecond = 0; //don't move robot
    }
    else if (timer.get() > Constants.ArmConstants.WaitScore && timer.get() < Constants.ArmConstants.DriveBackTime){
        arm.GripperSpitOut();
        scoreSpeeds.vxMetersPerSecond = Constants.ArmConstants.DriveBackSpeed; //move robot
    }
    else if (timer.get() > Constants.ArmConstants.DriveBackTime){
        arm.StopGripper();
        scoreSpeeds.vxMetersPerSecond = 0; //stop everything 
    }

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
    return timer.get() > Constants.ArmConstants.DriveBackTime;
  }
}
