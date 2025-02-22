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
public class TrackTagLeft extends Command {
  /** Creates a new TrackTagLeft. */
  public final SwerveSubsystem swerveDrive;
  
  public final CommandXboxController driverJoystick;

  private final ChassisSpeeds tagSpeeds;
  private final Timer timer;

  public double angleTarget;
  public double tagNumber;
  
  public TrackTagLeft(SwerveSubsystem m_swerveDrive, CommandXboxController m_driverJoystick) {
    // Use addRequirements() here to declare subsystem dependencies.
    swerveDrive = m_swerveDrive;
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
    angleTarget = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    tagNumber = swerveDrive.GetTagID();

    if (tagNumber == 18 || tagNumber == 7){
      angleTarget = 0;
    }
    if (tagNumber == 17 || tagNumber == 8){
      angleTarget = 60;
    }
    if (tagNumber == 22 || tagNumber == 9){
      angleTarget = 120;
    }
    if (tagNumber == 21 || tagNumber == 10){
      angleTarget = 180;
    }
    if (tagNumber == 20 || tagNumber == 11){
      angleTarget = 240;
    }
    if (tagNumber == 19 || tagNumber == 6){
      angleTarget = 300;
    }

    //define forward speed
    tagSpeeds.vxMetersPerSecond = Constants.DrivebaseConstants.ReefForwardSpeed;

    //define side-to-side speed
    tagSpeeds.vyMetersPerSecond = Constants.DrivebaseConstants.ReefKp*(swerveDrive.TrackReefTagX() + Constants.DrivebaseConstants.OffsetForLeft); //multiply Limelight value by P factor
    
    //define rotational speed
    tagSpeeds.omegaRadiansPerSecond = (angleTarget - swerveDrive.getHeading().getDegrees())*Constants.DrivebaseConstants.ReefSpinKp;

    //send values to swervedrive
    swerveDrive.drive(tagSpeeds);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return !driverJoystick.getHID().getLeftBumperButton(); //stop once left bumper released
  }
}
