// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsytem;
import frc.robot.subsystems.ElevatorSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class MoveToHangHeight extends Command {
  /** Creates a new ArmReady. */
  public final ElevatorSubsystem elevator;
  public final ArmSubsytem arm;
  public final CommandXboxController operatorJoystick;
  public final Timer timer;
 
  public MoveToHangHeight(ElevatorSubsystem m_elevator, ArmSubsytem m_arm, CommandXboxController m_operatorJoystick) {
    // Use addRequirements() here to declare subsystem dependencies.
    elevator = m_elevator;
    arm = m_arm;
    operatorJoystick = m_operatorJoystick;
    timer = new Timer();


    addRequirements(elevator, arm);
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
    //elevator movement
    if (arm.GetArmEncoderPosition() > Constants.ArmConstants.armFlat){
      elevator.StopElevator();
    }
    else if (arm.GetArmEncoderPosition() < Constants.ArmConstants.armFlat && elevator.GetElevatorEncoderPosition() > (Constants.ElevatorConstants.ShallowHangHeight + Constants.ElevatorConstants.deadband)){
      elevator.AutoElevator(Constants.ElevatorConstants.AutoDownSpeed);
    }
    else if (arm.GetArmEncoderPosition() < Constants.ArmConstants.armFlat && elevator.GetElevatorEncoderPosition() < (Constants.ElevatorConstants.ShallowHangHeight - Constants.ElevatorConstants.deadband)){
      //elevator.AutoElevator(Constants.ElevatorConstants.AutoUpSpeed);
      elevator.AutoElevator(Constants.ElevatorConstants.FasterUpSpeed);
    }
    //else elevator.StopElevator();
    else elevator.AutoElevator(Constants.ElevatorConstants.HoldElevatorSpeed);

    //arm movement
    if (arm.GetTopLimitSwitch() == false && arm.GetArmEncoderPosition() > Constants.ArmConstants.AlmostUpValue){
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
    timer.stop();
    elevator.StopElevator();
    arm.StopArm();

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return !operatorJoystick.getHID().getRawButton(7);
  
  }
}
