// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.MoveToL2;
import frc.robot.commands.MoveToL3;
import frc.robot.commands.MoveToL4;
import frc.robot.commands.ArmFlatToHang;
import frc.robot.commands.CenterTrack;
import frc.robot.commands.ChangePipeline;
import frc.robot.commands.GoBackUp;
import frc.robot.commands.HangCommand;
import frc.robot.commands.ManualArmCommand;
import frc.robot.commands.ManualChuteCommand;
import frc.robot.commands.ManualElevatorCommand;
import frc.robot.commands.MoveToL1;
import frc.robot.commands.PickFromTrough;
import frc.robot.commands.ScoreCoral;
import frc.robot.commands.ScoreCoralFast;
import frc.robot.commands.TrackHumanLoading;
import frc.robot.commands.TrackReefLeft;
import frc.robot.commands.TrackReefRight;
import frc.robot.commands.TrackTagLeft;
import frc.robot.commands.TrackTagRight;
import frc.robot.commands.TransferPosition;
import frc.robot.commands.Tuck;
import frc.robot.commands.AutoMode.AutoModeOpenChute;
import frc.robot.commands.AutoMode.AutoPick;
import frc.robot.commands.AutoMode.AutoScoreCoral;
import frc.robot.commands.AutoMode.AutoToL4;
import frc.robot.commands.AutoMode.AutoTransferPosition;
import frc.robot.commands.AutoMode.HangerSafeOut;
import frc.robot.commands.AutoMode.LeftTagTrackAuto;
import frc.robot.commands.AutoMode.RightTagTrackAuto;
import frc.robot.commands.AutoMode.TuckAuto;
import frc.robot.commands.ChangeTurningCommand;
import frc.robot.commands.DriveSideways;
import frc.robot.subsystems.ArmSubsytem;
import frc.robot.subsystems.ChuteSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.HangSubsystem;
//import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.io.File;

import swervelib.SwerveInputStream;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer
{
  SlewRateLimiter driveRateLimit;

  // Joysticks
  public static final CommandXboxController driverXbox = new CommandXboxController(Constants.OperatorConstants.DriverUSBPort);
  public static final CommandXboxController operatorXbox = new CommandXboxController(Constants.OperatorConstants.OperatorUSBPort);
  public static final Joystick panel = new Joystick(Constants.OperatorConstants.PanelUSBPort);

  //Auto Mode Chooser
  private final SendableChooser<Command> autoChooser;

  // Subsystems
  public static final SwerveSubsystem drivebase  = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),"swerve/neo"));
  private final ElevatorSubsystem elevator = new ElevatorSubsystem();
  private final ArmSubsytem arm = new ArmSubsytem();
  private final HangSubsystem hanger = new HangSubsystem();
  private final ChuteSubsystem chute = new ChuteSubsystem();
  //private final Limelight limelight = new Limelight();

  //Commands
  private final ManualElevatorCommand manualElevator;
  private final ManualArmCommand manualArm;
  private final TransferPosition transfer;
  private final MoveToL1 moveToL1;
  private final MoveToL2 moveToL2;
  private final MoveToL3 moveToL3;
  private final MoveToL4 moveToL4;
  private final ScoreCoral scoreCoral;
  private final ScoreCoralFast scoreFast;
  private final Tuck tuck;
  private final ArmFlatToHang flatArm;

  private final HangCommand hangCommand;

  private final DriveSideways driveSideways;

  //private final ChangePipeline changePipeline;
  //private final TrackTagLeft trackLeft;
  private final TrackReefLeft trackReefLeft;
  //private final TrackTagRight trackRight;
  private final TrackReefRight trackReefRight;
  private final TrackHumanLoading trackCoralStation;
  private final CenterTrack centerTrack;
  private final PickFromTrough pick;
  private final GoBackUp moveUp;
  private final ManualChuteCommand moveChute;
  private final ChangeTurningCommand changeTurning;

  private final SequentialCommandGroup autoTransfer;


  /**
   * Converts driver input into a field-relative ChassisSpeeds that is controlled by angular velocity.
   */
  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                                                () -> driverXbox.getLeftY() * -1,
                                                                () -> driverXbox.getLeftX() * -1)
                                                            .withControllerRotationAxis(driverXbox::getRightX)
                                                            .deadband(OperatorConstants.DEADBAND)
                                                            .scaleTranslation(0.8)
                                                            .allianceRelativeControl(true);

  /**
   * Clone's the angular velocity input stream and converts it to a fieldRelative input stream.
   */
  SwerveInputStream driveDirectAngle = driveAngularVelocity.copy().withControllerHeadingAxis(driverXbox::getRightX,
                                                                                             driverXbox::getRightY)
                                                           .headingWhile(true);

  /**
   * Clone's the angular velocity input stream and converts it to a robotRelative input stream.
   */
  SwerveInputStream driveRobotOriented = driveAngularVelocity.copy().robotRelative(true)
                                                             .allianceRelativeControl(false);


  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer()
  {
    driveRateLimit = new SlewRateLimiter(Constants.DrivebaseConstants.DriveRateLimit);

  
    //PathPlanner Named Commands
    NamedCommands.registerCommand("Move to L4", new AutoToL4(elevator, arm));
    NamedCommands.registerCommand("Move to Human Loading", new AutoTransferPosition(elevator, arm));
    NamedCommands.registerCommand("Grab Coral", new AutoPick(elevator, arm));
    NamedCommands.registerCommand("Score Coral", new AutoScoreCoral(elevator, arm, drivebase));
    NamedCommands.registerCommand("Track Left Pipe", new LeftTagTrackAuto(drivebase));
    NamedCommands.registerCommand("Track Right Pipe", new RightTagTrackAuto(drivebase));
    NamedCommands.registerCommand("Tuck Arm", new TuckAuto(elevator, arm));
    NamedCommands.registerCommand("Move Hang Arm A Bit", new HangerSafeOut(hanger));
    NamedCommands.registerCommand("Open Chute For Automode", new AutoModeOpenChute(chute));

    //Commands
    manualElevator = new ManualElevatorCommand(elevator, operatorXbox);
    manualArm = new ManualArmCommand(arm, operatorXbox);

    //changePipeline = new ChangePipeline(limelight, driverXbox);
    //trackLeft = new TrackTagLeft(drivebase, driverXbox);
    trackReefLeft = new TrackReefLeft(drivebase, driverXbox);
    //trackRight = new TrackTagRight(drivebase, driverXbox);
    trackReefRight = new TrackReefRight(drivebase, driverXbox);
    trackCoralStation = new TrackHumanLoading(drivebase, driverXbox);
    centerTrack = new CenterTrack(drivebase, driverXbox);
    changeTurning = new ChangeTurningCommand(drivebase, driverXbox);
    driveSideways = new DriveSideways(drivebase, driverXbox);

    hangCommand = new HangCommand(hanger, operatorXbox);

    transfer = new TransferPosition(elevator, arm, panel);
    moveToL1 = new MoveToL1(elevator, arm, panel);
    moveToL2 = new MoveToL2(elevator, arm, panel);
    moveToL3 = new MoveToL3(elevator, arm, panel);
    moveToL4 = new MoveToL4(elevator, arm, panel);
    scoreCoral = new ScoreCoral(elevator, arm, drivebase, panel);
    scoreFast = new ScoreCoralFast(elevator, arm, drivebase, panel);
    tuck = new Tuck(elevator, arm, panel);
    flatArm = new ArmFlatToHang(elevator, arm, operatorXbox);
    pick = new PickFromTrough(elevator, arm, panel);
    moveUp = new GoBackUp(elevator, arm, panel);
    
    moveChute = new ManualChuteCommand(chute, operatorXbox);
    autoTransfer = new SequentialCommandGroup(pick, moveUp); //sequential command group for auto transfer

    //Automode Chooser
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Mode Choice", autoChooser);

    // Configure the trigger bindings
    configureBindings();
    DriverStation.silenceJoystickConnectionWarning(true);
    NamedCommands.registerCommand("test", Commands.print("I EXIST"));

    Command standardDrive = drivebase.driveCommand(
        () -> MathUtil.applyDeadband(-driverXbox.getLeftY()*elevator.IsElevatorTooHigh(), OperatorConstants.DEADBAND),
        () -> MathUtil.applyDeadband(-driveRateLimit.calculate(driverXbox.getLeftX())*elevator.IsElevatorTooHigh(), OperatorConstants.DEADBAND),
        () -> MathUtil.applyDeadband(-Constants.DrivebaseConstants.SlowDownTurn*driverXbox.getRightX(), OperatorConstants.DEADBAND));

    Command driveFieldOrientedDirectAngle      = drivebase.driveFieldOriented(driveDirectAngle);
    Command driveFieldOrientedAnglularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);
    Command driveRobotOrientedAngularVelocity  = drivebase.driveFieldOriented(driveRobotOriented);
    
    //Default Commands for each subsystem
    drivebase.setDefaultCommand(standardDrive);
    elevator.setDefaultCommand(manualElevator);
    arm.setDefaultCommand(manualArm);
    hanger.setDefaultCommand(hangCommand);
    chute.setDefaultCommand(moveChute);
    //limelight.setDefaultCommand(changePipeline);
  }

  private void configureBindings()
  {
    //Buttons   
      driverXbox.start().onTrue((Commands.runOnce(drivebase::zeroGyro)));
      //new JoystickButton(driverXbox, 8).onTrue(new InstantCommand(drivebase::zeroGyro));
      driverXbox.back().onTrue(Commands.none());
      driverXbox.x().onTrue(trackReefLeft);
      driverXbox.b().onTrue(trackReefRight);
      driverXbox.a().onTrue(trackCoralStation);
      driverXbox.y().onTrue(centerTrack);
      driverXbox.leftBumper().onTrue(driveSideways);
      driverXbox.rightBumper().onTrue(driveSideways);

      operatorXbox.y().onTrue(flatArm);

      new JoystickButton(panel, Constants.OperatorConstants.CoralStationButton).onTrue(transfer);
      new JoystickButton(panel, Constants.OperatorConstants.GetCoralButton).onTrue(autoTransfer);
      new JoystickButton(panel, Constants.OperatorConstants.TuckArmButton).onTrue(tuck);
      new JoystickButton(panel, Constants.OperatorConstants.ScoreCoralButton).onTrue(scoreCoral);
      new JoystickButton(panel, Constants.OperatorConstants.ScoreAndBackUpFastButton).onTrue(scoreFast);
      
      new JoystickButton(panel, Constants.OperatorConstants.L1Button).onTrue(moveToL1);
      new JoystickButton(panel, Constants.OperatorConstants.L2Button).onTrue(moveToL2);
      new JoystickButton(panel, Constants.OperatorConstants.L3Button).onTrue(moveToL3);
      new JoystickButton(panel, Constants.OperatorConstants.L4Button).onTrue(moveToL4);




  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand()
  {
    //return autoChooser.getSelected();
    //return new PathPlannerAuto("Left Start - Score 2 Coral");//two Coral from left start
    return new PathPlannerAuto("Left Start - Score Coral");//Single Coral from left start
    //return new PathPlannerAuto("Left Start - Hide");//Just cross the line and do nothing
    
    //return new PathPlannerAuto("Right Start - Score Coral");//Single Coral from right start
    //return new PathPlannerAuto("Right Start - Hide");//Just cross the line and do nothing
    
    //return new PathPlannerAuto("Center Start - Score Coral");//Single Coral starting at center
  }

  public void setMotorBrake(boolean brake)
  {
    drivebase.setMotorBrake(brake);
  }
}
