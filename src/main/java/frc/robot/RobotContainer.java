// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.MathUtil;
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
import frc.robot.commands.ChangePipeline;
import frc.robot.commands.GoBackUp;
import frc.robot.commands.ManualArmCommand;
import frc.robot.commands.ManualElevatorCommand;
import frc.robot.commands.MoveToL1;
import frc.robot.commands.PickFromTrough;
import frc.robot.commands.ScoreCoral;
import frc.robot.commands.TrackTagLeft;
import frc.robot.commands.TrackTagRight;
import frc.robot.commands.TransferPosition;
import frc.robot.commands.Tuck;
import frc.robot.commands.AutoMode.AutoPick;
import frc.robot.commands.AutoMode.AutoScoreCoral;
import frc.robot.commands.AutoMode.AutoToL4;
import frc.robot.commands.AutoMode.AutoTransferPosition;
import frc.robot.commands.AutoMode.LeftTagTrackAuto;
import frc.robot.commands.ChangeTurningCommand;
import frc.robot.commands.DriveSideways;
import frc.robot.subsystems.ArmSubsytem;
import frc.robot.subsystems.ElevatorSubsystem;
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
  private final Tuck tuck;

  private final DriveSideways driveSideways;

  //private final ChangePipeline changePipeline;
  private final TrackTagLeft trackLeft;
  private final TrackTagRight trackRight;
  private final PickFromTrough pick;
  private final GoBackUp moveUp;
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
  
    //PathPlanner Named Commands
    NamedCommands.registerCommand("Move to L4", new AutoToL4(elevator, arm));
    NamedCommands.registerCommand("Move to Human Loading", new AutoTransferPosition(elevator, arm));
    NamedCommands.registerCommand("Grab Coral", new AutoPick(elevator, arm));
    NamedCommands.registerCommand("Score Coral", new AutoScoreCoral(elevator, arm, drivebase));
    NamedCommands.registerCommand("Track Left Pipe", new LeftTagTrackAuto(drivebase));

    //Commands
    manualElevator = new ManualElevatorCommand(elevator, operatorXbox);
    manualArm = new ManualArmCommand(arm, operatorXbox);

    //changePipeline = new ChangePipeline(limelight, driverXbox);
    trackLeft = new TrackTagLeft(drivebase, driverXbox);
    trackRight = new TrackTagRight(drivebase, driverXbox);
    changeTurning = new ChangeTurningCommand(drivebase, driverXbox);
    driveSideways = new DriveSideways(drivebase, driverXbox);

    transfer = new TransferPosition(elevator, arm, panel);
    moveToL1 = new MoveToL1(elevator, arm, panel);
    moveToL2 = new MoveToL2(elevator, arm, panel);
    moveToL3 = new MoveToL3(elevator, arm, panel);
    moveToL4 = new MoveToL4(elevator, arm, panel);
    scoreCoral = new ScoreCoral(elevator, arm, drivebase, panel);
    tuck = new Tuck(elevator, arm, panel);
    pick = new PickFromTrough(elevator, arm, panel);
    moveUp = new GoBackUp(elevator, arm, panel);
    autoTransfer = new SequentialCommandGroup(pick, moveUp); //sequential command group for auto transfer

    //Automode Chooser
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Mode Choice", autoChooser);

    // Configure the trigger bindings
    configureBindings();
    DriverStation.silenceJoystickConnectionWarning(true);
    NamedCommands.registerCommand("test", Commands.print("I EXIST"));

    Command standardDrive = drivebase.driveCommand(
        () -> MathUtil.applyDeadband(-driverXbox.getLeftY(), OperatorConstants.DEADBAND),
        () -> MathUtil.applyDeadband(-driverXbox.getLeftX(), OperatorConstants.DEADBAND),
        () -> MathUtil.applyDeadband(Constants.DrivebaseConstants.SlowDownTurn*-driverXbox.getRightX(), OperatorConstants.DEADBAND));

    Command driveFieldOrientedDirectAngle      = drivebase.driveFieldOriented(driveDirectAngle);
    Command driveFieldOrientedAnglularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);
    Command driveRobotOrientedAngularVelocity  = drivebase.driveFieldOriented(driveRobotOriented);
    
    //Default Commands for each subsystem
    drivebase.setDefaultCommand(standardDrive);
    elevator.setDefaultCommand(manualElevator);
    arm.setDefaultCommand(manualArm);
    //limelight.setDefaultCommand(changePipeline);
  }

  private void configureBindings()
  {
    //Buttons   
      driverXbox.start().onTrue((Commands.runOnce(drivebase::zeroGyro)));
      //new JoystickButton(driverXbox, 8).onTrue(new InstantCommand(drivebase::zeroGyro));
      driverXbox.back().onTrue(Commands.none());
      driverXbox.x().onTrue(trackLeft);
      driverXbox.b().onTrue(trackRight);
      driverXbox.leftBumper().onTrue(driveSideways);
      driverXbox.rightBumper().onTrue(driveSideways);
      driverXbox.rightBumper().onTrue(Commands.none());

      new JoystickButton(panel, Constants.OperatorConstants.CoralStationButton).onTrue(transfer);
      new JoystickButton(panel, Constants.OperatorConstants.GetCoralButton).onTrue(autoTransfer);
      new JoystickButton(panel, Constants.OperatorConstants.TuckArmButton).onTrue(tuck);
      new JoystickButton(panel, Constants.OperatorConstants.ScoreCoralButton).onTrue(scoreCoral);
      
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
    return autoChooser.getSelected();
  }

  public void setMotorBrake(boolean brake)
  {
    drivebase.setMotorBrake(brake);
  }
}
