// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoMode;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsytem;
import frc.robot.subsystems.ElevatorSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AutoPick extends Command {
  /** Creates a new PickFromTrough. */
  public final ElevatorSubsystem elevator;
  public final ArmSubsytem arm;
  
  public AutoPick(ElevatorSubsystem m_elevator, ArmSubsytem m_arm) {
    // Use addRequirements() here to declare subsystem dependencies.
    elevator = m_elevator;
    arm = m_arm;

    addRequirements(elevator, arm);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (elevator.GetElevatorEncoderPosition() > Constants.ElevatorConstants.troughHeight){
      elevator.AutoElevator(Constants.ElevatorConstants.BumpDownSpeed);
      arm.GripperIntake();
    }
    else{
      elevator.StopElevator();
      arm.StopGripper();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return elevator.GetElevatorEncoderPosition() < Constants.ElevatorConstants.troughHeight;
  }
}
