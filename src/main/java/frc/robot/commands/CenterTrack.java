// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class CenterTrack extends Command {
  /** Creates a new CenterTrack. */

  public final SwerveSubsystem swerveDrive;
  public final CommandXboxController driverJoystick;

  private final ChassisSpeeds tagSpeeds;
  private final Timer timer;

  public double tagNumberCenter;

  public CenterTrack(SwerveSubsystem m_swerve, CommandXboxController m_driverJoystick) {
    // Use addRequirements() here to declare subsystem dependencies.
    swerveDrive = m_swerve;
    driverJoystick = m_driverJoystick;

    timer = new Timer();
    tagSpeeds = new ChassisSpeeds(0,0,0);

    addRequirements(swerveDrive);

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

  tagNumberCenter = swerveDrive.GetTagID();
        //define forward speed
    if (swerveDrive.TrackReefTagY() > Constants.DrivebaseConstants.tagHeight && tagNumberCenter != -1){
    tagSpeeds.vxMetersPerSecond = Constants.DrivebaseConstants.ReefForwardSpeed;
    }
    else tagSpeeds.vxMetersPerSecond = 0;

    //define side-to-side speed
    if (tagNumberCenter != -1 && swerveDrive.TrackReefTagY() < 0){
      tagSpeeds.vyMetersPerSecond = Constants.DrivebaseConstants.ReefKp*(swerveDrive.TrackReefTagX() + Constants.DrivebaseConstants.OffsetForCenter); //multiply Limelight value by P factor
    }
    else tagSpeeds.vyMetersPerSecond = 0;

    //define rotational speed
    tagSpeeds.omegaRadiansPerSecond = 0;

    //send values to swervedrive
    swerveDrive.drive(tagSpeeds);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return !driverJoystick.getHID().getAButton(); //stop once A button released released;
  }
}
