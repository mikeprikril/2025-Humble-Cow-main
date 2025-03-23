// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoMode;
import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.HangSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class HangerSafeOut extends Command {
  public final HangSubsystem hang;

  /** Creates a new HangerSafeOut. */
  public HangerSafeOut(HangSubsystem m_hang) {
    // Use addRequirements() here to declare subsystem dependencies.
    hang = m_hang;

    addRequirements(hang);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    hang.HangerJustABitOut();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    hang.StopHangMotor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return hang.GetHangEncoderPosition() > Constants.HangConstants.HangerJustABitOut;
  }
}
