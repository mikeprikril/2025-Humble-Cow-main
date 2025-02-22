// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoMode;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsytem;
import frc.robot.subsystems.ElevatorSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AutoToL4 extends Command {
  /** Creates a new AutoToL4. */
  public final ElevatorSubsystem elevator;
  public final ArmSubsytem arm;

 
  public AutoToL4(ElevatorSubsystem m_elevator, ArmSubsytem m_arm) {
    // Use addRequirements() here to declare subsystem dependencies.
    elevator = m_elevator;
    arm = m_arm;


    addRequirements(elevator, arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //elevator movement

    if (elevator.GetElevatorEncoderPosition() < Constants.ElevatorConstants.AlmostUpValue){
      elevator.AutoElevator(Constants.ElevatorConstants.AutoUpSpeed);
    }
    else if (elevator.GetElevatorEncoderPosition() > Constants.ElevatorConstants.AlmostUpValue && elevator.GetElevatorEncoderPosition() < Constants.ElevatorConstants.UpLimit){
      elevator.AutoElevator(Constants.ElevatorConstants.AutoUpSpeed*.3);
    }
    else elevator.StopElevator();

    //arm movement
    if (arm.GetArmEncoderPosition() > Constants.ArmConstants.AlmostUpValue){
      arm.AutoArmMove(Constants.ArmConstants.ArmUpFast);
    }
    else if (arm.GetTopLimitSwitch() == false && arm.GetArmEncoderPosition() < Constants.ArmConstants.AlmostUpValue){
      arm.AutoArmMove(Constants.ArmConstants.ArmUpSpeed*Constants.ArmConstants.SlowDown);
    }
    else arm.StopArm();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    elevator.StopElevator();
    arm.StopArm();

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return elevator.GetTopLimitSwitch() == true && arm.GetTopLimitSwitch() == true;
  }
}
