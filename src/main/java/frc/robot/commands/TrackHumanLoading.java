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
public class TrackHumanLoading extends Command {
  /** Creates a new TrackTagLeft. */
  public final SwerveSubsystem swerveDrive;
  
  public final CommandXboxController driverJoystick;

  private final ChassisSpeeds loadingSpeeds;
  private final Timer timer;

  public double angleTarget;
  public double tagNumber;
  
  public TrackHumanLoading(SwerveSubsystem m_swerveDrive, CommandXboxController m_driverJoystick) {
    // Use addRequirements() here to declare subsystem dependencies.
    swerveDrive = m_swerveDrive;
    driverJoystick = m_driverJoystick;

    timer = new Timer();
    loadingSpeeds = new ChassisSpeeds(0,0,0);

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
    tagNumber = swerveDrive.GetHumanTagID();

    if (tagNumber == 12 || tagNumber == 2){
      angleTarget = 52.6;
    }
    if (tagNumber == 13 || tagNumber == 1){
      angleTarget = -52.6;
    }

    //define forward speed
    //loadingSpeeds.vxMetersPerSecond = -driverJoystick.getLeftY()*Constants.DrivebaseConstants.HumanLoadingKp; //pass values from joystick
    if (tagNumber != -1 && swerveDrive.TrackHumanTagY() < 6){
  loadingSpeeds.vxMetersPerSecond = Constants.DrivebaseConstants.CoralStationDriveBackSpeed;
    }
    else loadingSpeeds.vxMetersPerSecond = 0;

    //define side-to-side speed
    //loadingSpeeds.vyMetersPerSecond = -driverJoystick.getLeftX()*Constants.DrivebaseConstants.HumanLoadingKp; //pass values from joystick
    if (tagNumber != -1){
      //loadingSpeeds.vyMetersPerSecond = Constants.DrivebaseConstants.CoralStationSideKp*(swerveDrive.TrackHumanTagX() + Constants.DrivebaseConstants.CoralStationOffset);
    }
    else loadingSpeeds.vyMetersPerSecond = 0;

    //define rotational speed
    if (tagNumber != -1){
      loadingSpeeds.omegaRadiansPerSecond = (angleTarget - swerveDrive.getHeading().getDegrees())*Constants.DrivebaseConstants.CoralStationSpinKp;
    }
    else loadingSpeeds.omegaRadiansPerSecond = 0;

    //send values to swervedrive
    swerveDrive.drive(loadingSpeeds); 
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return !driverJoystick.getHID().getAButton(); //stop once A button released released
  }
}
