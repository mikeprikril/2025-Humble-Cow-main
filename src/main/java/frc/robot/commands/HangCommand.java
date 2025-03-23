// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.HangSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class HangCommand extends Command {
  /** Creates a new HangCommand. */
  public final HangSubsystem hang;
  public final CommandXboxController operatorController;


  public HangCommand(HangSubsystem m_hang, CommandXboxController m_operatorController) {
    // Use addRequirements() here to declare subsystem dependencies.
    hang = m_hang;
    operatorController = m_operatorController;

    addRequirements(hang);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (operatorController.getHID().getRawButton(8) == true){
      hang.HangRobot();
    }
    else if (operatorController.getHID().getRawButton(7) == true){
      hang.ExtendHanger();
    }
    else if (operatorController.getHID().getXButton()){
      hang.HangerJustABitOut();
    }
    else if (operatorController.getHID().getBButton()){
      hang.HangerVertical();
    }
    else hang.StopHangMotor();

  if (operatorController.getHID().getRawButton(2) == true){
      hang.RotateHangBar(.5);
    }
    else hang.RotateHangBar(10);

  if (operatorController.getHID().getAButton() == true){
    hang.ResetHangEncoder();
  }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
