// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;



/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ChangePipeline extends Command {
  /** Creates a new ChangePipeline. */
  //private final Limelight limelight;
  public final CommandXboxController driverJoystick;

  public ChangePipeline(CommandXboxController m_driverJoystick) {
    // Use addRequirements() here to declare subsystem dependencies.
    //limelight = m_limelight;
    driverJoystick = m_driverJoystick;

    //addRequirements(limelight);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //limelight.SetPipeline(Constants.LimelightConstants.AprilTagPipeline); //set to apriltag when robot starts
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if (driverJoystick.getHID().getAButton() == true) {
      //limelight.SetPipeline(Constants.LimelightConstants.AprilTagPipeline);
    }

    if (driverJoystick.getHID().getBButton() == true) {
      //limelight.SetPipeline(Constants.LimelightConstants.ColorPipeline);
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
