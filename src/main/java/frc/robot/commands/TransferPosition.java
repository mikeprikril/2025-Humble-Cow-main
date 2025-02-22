// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
//import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsytem;
import frc.robot.subsystems.ElevatorSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class TransferPosition extends Command {

  public final ElevatorSubsystem elevator;
  public final ArmSubsytem arm;
  //public final CommandXboxController operatorJoystick;
  public final Joystick panel;

  public TransferPosition(ElevatorSubsystem m_elevator, ArmSubsytem m_arm, Joystick m_panel) {
    // Use addRequirements() here to declare subsystem dependencies.
    elevator = m_elevator;
    arm = m_arm;
    panel = m_panel;
    //operatorJoystick = m_operatorJoystick;


    addRequirements(elevator, arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (elevator.GetElevatorEncoderPosition() < (Constants.ElevatorConstants.TransferHeight - Constants.ElevatorConstants.deadband)) { //is elevator below target
      elevator.AutoElevator(Constants.ElevatorConstants.AutoUpSpeed);

    }
    else if (elevator.GetElevatorEncoderPosition() > (Constants.ElevatorConstants.TransferHeight + Constants.ElevatorConstants.deadband)){ //is elevator above target
      elevator.AutoElevator(Constants.ElevatorConstants.AutoDownSpeed);
    }
    else {
      elevator.AutoElevator(Constants.ElevatorConstants.HoldElevatorSpeed); //if at target then hold
    }

    if (elevator.GetElevatorEncoderPosition() < Constants.ElevatorConstants.SafeHeight){
      arm.StopArm();
    } 
    else if (arm.GetArmEncoderPosition() < (Constants.ArmConstants.ArmAtLoading - Constants.ArmConstants.CloseSlow)) { //arm is not near bottom
        arm.AutoArmMove(Constants.ArmConstants.ArmDownSpeed);
      }
    else if (arm.GetArmEncoderPosition() > (Constants.ArmConstants.ArmAtLoading - Constants.ArmConstants.CloseSlow) && (arm.GetArmEncoderPosition() < Constants.ArmConstants.ArmAtLoading)){
        arm.AutoArmMove(Constants.ArmConstants.ArmDownSpeed*Constants.ArmConstants.SlowDown);
      }
    else {
        arm.StopArm(); //if at limit switch then stop motor
      } 
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    arm.StopArm();
    elevator.StopElevator();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return !panel.getRawButton(Constants.OperatorConstants.CoralStationButton); //stop command if you let go of this button
    //!operatorJoystick.getHID().getXButton(); // stop command if button let go
    /*|| 
    (
    ((elevator.GetElevatorEncoderPosition() > (Constants.ElevatorConstants.TransferHeight - Constants.ElevatorConstants.deadband)) 
    &&
    (elevator.GetElevatorEncoderPosition() < (Constants.ElevatorConstants.TransferHeight + Constants.ElevatorConstants.deadband)) 
    )
    &&
    (
      (arm.GetArmEncoderPosition() > Constants.ArmConstants.ArmAtLoading)
    )
    ); */
  }
}
