// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DriveSideways extends Command {
  /** Creates a new DriveSideways. */
  public final SwerveSubsystem swerve;
  public final CommandXboxController driverController;
  public final ChassisSpeeds driveSpeeds;

  public DriveSideways(SwerveSubsystem m_swerve, CommandXboxController m_drivController) {
    // Use addRequirements() here to declare subsystem dependencies.
    swerve = m_swerve;
    driverController = m_drivController;

    driveSpeeds = new ChassisSpeeds(0, 0, 0);

    addRequirements(swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    driveSpeeds.vxMetersPerSecond = 0;
    driveSpeeds.omegaRadiansPerSecond = 0;


    if (driverController.getHID().getRawButton(5) == true){
      driveSpeeds.vyMetersPerSecond = Constants.DrivebaseConstants.CreepSpeed;
    }
    else if (driverController.getHID().getRawButton(6) == true){
      driveSpeeds.vyMetersPerSecond = -Constants.DrivebaseConstants.CreepSpeed;
    }
    else driveSpeeds.vyMetersPerSecond = 0;

  swerve.drive(driveSpeeds);

    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return !driverController.getHID().getRawButton(5) && !driverController.getHID().getRawButton(6);
  }
}
