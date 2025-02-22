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
public class GoBackUp extends Command {
  /** Creates a new PickFromTrough. */
  public final ElevatorSubsystem elevator;
  public final ArmSubsytem arm;
  public final Joystick panel;
  //public final CommandXboxController operatorJoystick;
  
  public GoBackUp(ElevatorSubsystem m_elevator, ArmSubsytem m_arm, Joystick m_panel) {
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
    if (elevator.GetElevatorEncoderPosition() < Constants.ElevatorConstants.TransferHeight){
      elevator.AutoElevator(Constants.ElevatorConstants.AutoUpSpeed/2);
      arm.AutoArmMove(Constants.ArmConstants.ArmUpSpeed*.25);//arm up at half speed
    }
    else{
      elevator.StopElevator();
      arm.StopArm();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return !panel.getRawButton(Constants.OperatorConstants.GetCoralButton);
    //!operatorJoystick.getHID().getYButton();
  }
}
