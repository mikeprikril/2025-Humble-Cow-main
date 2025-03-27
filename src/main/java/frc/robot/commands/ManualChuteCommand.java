// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.ChuteSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ManualChuteCommand extends Command {
  /** Creates a new ManualChuteCommand. */
public final ChuteSubsystem chute;
public final CommandXboxController operatorController;

  public ManualChuteCommand(ChuteSubsystem m_chute, CommandXboxController m_operatorController) {
    // Use addRequirements() here to declare subsystem dependencies.
    chute = m_chute;
    operatorController = m_operatorController;

    addRequirements(chute);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (operatorController.getHID().getRawButton(1) == true){
      chute.ServoCloseChute();
    }
    else chute.ServoOpenChute();
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
