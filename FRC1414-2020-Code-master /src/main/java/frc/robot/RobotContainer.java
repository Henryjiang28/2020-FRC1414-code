/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.ControlPanel.ControlPanelAutoCmd;
import frc.robot.commands.ControlPanel.ControlPanelTeleopCmd;
import frc.robot.commands.Drivetrain.DriveAutoCmd;
import frc.robot.commands.Drivetrain.DrivePathCmd;
import frc.robot.commands.Drivetrain.DriveTeleopCmd;
import frc.robot.commands.Hood.HoodAutoCmd;
import frc.robot.commands.Hood.HoodTeleopCmd;
import frc.robot.commands.Indexer.IndexerTeleopCmd;
import frc.robot.commands.Turret.TurretAutoCmd;
import frc.robot.commands.Turret.TurretTeleopCmd;
import frc.robot.commands.Intake.IntakeTeleopCmd;
import frc.robot.commands.Shooter.ShooterTeleopCmd;
import frc.robot.subsystems.Turret;
import frc.robot.util.GenericPaths;
import frc.robot.subsystems.ControlPanel;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Indexer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  private final Drivetrain drivetrain = new Drivetrain();
  private final Turret turret = new Turret();
  private final Intake intake = new Intake();
  private final Indexer indexer = new Indexer();
  private final Hood hood = new Hood();
  // private final ControlPanel controlPanel = new ControlPanel();
  private final Shooter shooter = new Shooter();

  private SendableChooser<Command> chooser = new SendableChooser<>();

  public final XboxController driveController = new XboxController(Constants.DRIVE_CONTROLLER_ID);
  public final XboxController operatorController = new XboxController(Constants.OPERATOR_CONTROLLER_ID);
  // public final Joystick backupJoystick = new Joystick(Constants.BACKUP_JOYSTICK_ID);

  public RobotContainer() {
    this.turret.setVisionMode(true);

    this.drivetrain.setDefaultCommand(new DriveTeleopCmd(this.drivetrain,
      () -> this.driveController.getTriggerAxis(Hand.kRight) - this.driveController.getTriggerAxis(Hand.kLeft),
      () -> RobotContainer.stickDeadband(this.driveController.getX(Hand.kLeft), Constants.DRIVE_STICK_DEADBAND, 0), 
      () -> this.driveController.getAButton()
    ));

    this.intake.setDefaultCommand(new IntakeTeleopCmd(this.intake,
      () -> this.operatorController.getTriggerAxis(Hand.kLeft),
      () -> this.operatorController.getBumper(Hand.kLeft)
    ));

    this.indexer.setDefaultCommand(new IndexerTeleopCmd(this.indexer,
      () -> this.operatorController.getTriggerAxis(Hand.kLeft),
      () -> this.shooter.getActualRPM(),
      () -> this.operatorController.getBumper(Hand.kLeft),
      () -> this.operatorController.getAButton(),
      () -> this.operatorController.getBButton(),
      () -> this.operatorController.getTriggerAxis(Hand.kRight)
    ));
    
    this.turret.setDefaultCommand(new TurretTeleopCmd(this.turret,
      () -> this.operatorController.getX(Hand.kRight),
      () -> this.operatorController.getBumper(Hand.kRight)
    ));

    this.hood.setDefaultCommand(new HoodTeleopCmd(this.hood,
      () -> this.operatorController.getY(Hand.kLeft),
      () -> this.operatorController.getBumper(Hand.kRight)
    ));

    this.shooter.setDefaultCommand(new ShooterTeleopCmd(this.shooter,
      () -> this.operatorController.getTriggerAxis(Hand.kRight),
      () -> this.operatorController.getBButton()
    ));



    // this.controlPanel.setDefaultCommand(new ControlPanelTeleopCmd(this.controlPanel));

    this.chooser.setDefaultOption("Wait Command", new WaitCommand(15));
    this.chooser.setDefaultOption("Drive", new DriveAutoCmd(this.drivetrain, 1.0));
    this.chooser.addOption("Shoot/Drive/Intake", new SequentialCommandGroup(
                                              new ParallelCommandGroup(new TurretAutoCmd(this.turret).withTimeout(9), new HoodAutoCmd(this.hood, () -> this.shooter.getActualRPM()).withTimeout(9), new ShooterTeleopCmd(this.shooter, () -> 1.0, () -> false).withTimeout(9), new IndexerTeleopCmd(this.indexer, () -> 0.0, () -> this.shooter.getActualRPM(), () -> false, () -> true, () -> false, () -> 1.0).withTimeout(9), new RunCommand(() -> this.intake.setForward(), this.intake).withTimeout(9)),
                                              new ParallelCommandGroup(new DriveAutoCmd(this.drivetrain, -4.0).withTimeout(5), new HoodTeleopCmd(this.hood, () -> 0.0, () -> false).withTimeout(5), new IntakeTeleopCmd(this.intake, () -> 1.0, () -> false).withTimeout(5), new IndexerTeleopCmd(this.indexer, () -> 1.0, () -> this.shooter.getActualRPM(), () -> false, () -> false, () -> false, () -> 0.0).withTimeout(5))
                                            ));
    this.chooser.addOption("Shoot/Drive/Intake/Shoot", new SequentialCommandGroup(
      new ParallelCommandGroup(new TurretAutoCmd(this.turret).withTimeout(7), new HoodAutoCmd(this.hood, () -> this.shooter.getActualRPM()).withTimeout(7), new ShooterTeleopCmd(this.shooter, () -> 1.0, () -> false).withTimeout(7), new IndexerTeleopCmd(this.indexer, () -> 0.0, () -> this.shooter.getActualRPM(), () -> false, () -> true, () -> false, () -> 1.0).withTimeout(7), new RunCommand(() -> this.intake.setForward(), this.intake).withTimeout(7)),
      new ParallelCommandGroup(new DriveAutoCmd(this.drivetrain, -4.0).withTimeout(7), new TurretAutoCmd(this.turret).withTimeout(7), new HoodAutoCmd(this.hood, () -> this.shooter.getActualRPM()).withTimeout(7), new ShooterTeleopCmd(this.shooter, () -> 1.0, () -> false).withTimeout(7),  new IntakeTeleopCmd(this.intake, () -> 1.0, () -> false).withTimeout(7), new IndexerTeleopCmd(this.indexer, () -> 0.0, () -> this.shooter.getActualRPM(), () -> false, () -> true, () -> false, () -> 1.0).withTimeout(7))
    ));
                                            
    this.chooser.addOption("Shoot/Drive", new SequentialCommandGroup(
      new ParallelCommandGroup(new TurretAutoCmd(this.turret).withTimeout(9), new HoodAutoCmd(this.hood, () -> this.shooter.getActualRPM()).withTimeout(9), new ShooterTeleopCmd(this.shooter, () -> 1.0, () -> false).withTimeout(9), new IndexerTeleopCmd(this.indexer, () -> 0.0, () -> this.shooter.getActualRPM(), () -> false, () -> true, () -> false, () -> 1.0).withTimeout(9), new RunCommand(() -> this.intake.setForward(), this.intake).withTimeout(9)),
      new ParallelCommandGroup(new DriveAutoCmd(this.drivetrain, 1.0).withTimeout(5), new HoodTeleopCmd(this.hood, () -> 0.0, () -> false).withTimeout(5), new IntakeTeleopCmd(this.intake, () -> 1.0, () -> false).withTimeout(5), new IndexerTeleopCmd(this.indexer, () -> 1.0, () -> this.shooter.getActualRPM(), () -> false, () -> false, () -> false, () -> 0.0).withTimeout(5))
    ));
    this.chooser.addOption("Shoot", new ParallelCommandGroup(new TurretAutoCmd(this.turret).withTimeout(15), new HoodAutoCmd(this.hood, () -> this.shooter.getActualRPM()).withTimeout(15), new ShooterTeleopCmd(this.shooter, () -> 1.0, () -> false).withTimeout(15), new IndexerTeleopCmd(this.indexer, () -> 0.0, () -> this.shooter.getActualRPM(), () -> false, () -> true, () -> false, () -> 1.0).withTimeout(15), new RunCommand(() -> this.intake.setForward(), this.intake).withTimeout(15)));
    // this.chooser.addOption("Motion Profile", new DrivePathCmd(this.drivetrain, GenericPaths.path));

    SmartDashboard.putData("Auto Chooser", this.chooser);

    configureButtonBindings();
  }

  public static double stickDeadband(final double value, final double deadband, final double center) {
    return (value < (center + deadband) && value > (center - deadband)) ? center : value;
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by instantiating a {@link GenericHID} or one of its subclasses
   * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
   * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */

  private void configureButtonBindings() {
    new JoystickButton(this.driveController, Button.kX.value).whenPressed(() -> this.drivetrain.setMaxOutput(0.5))
      .whenReleased(() -> this.drivetrain.setMaxOutput(1.0));

    // new JoystickButton(this.driveController, Button.kB.value).whileHeld(() -> this.drivetrain.driveStraight(this.driveController.getTriggerAxis(Hand.kRight) - this.driveController.getTriggerAxis(Hand.kLeft), this.drivetrain.getGyroAngle()), this.drivetrain);
    // new JoystickButton(this.driveController, Button.kY.value).whileHeld(() -> this.drivetrain.turnToAngle(30), this.drivetrain);

    // new JoystickButton(this.backupJoystick, 7).whileHeld(() -> this.hood.setAngle(20), this.hood);

    new JoystickButton(this.operatorController, Button.kX.value).whenPressed(() -> this.intake.setForward(), this.intake);
    new JoystickButton(this.operatorController, Button.kY.value).whenPressed(() -> this.intake.setReverse(), this.intake);

    new JoystickButton(this.operatorController, Button.kA.value).whileHeld(new HoodAutoCmd(this.hood, () -> this.shooter.getActualRPM()));
    // new JoystickButton(this.operatorController, Button.kA.value).whileHeld(new TurretAutoCmd(this.turret));

    new JoystickButton(this.operatorController, Button.kB.value).whileHeld(() -> this.hood.lowGoal(), this.hood);
    new JoystickButton(this.operatorController, Button.kB.value).whileHeld(() -> this.turret.lowGoal(), this.turret);

    // new JoystickButton(this.operatorController, Button.kB.value).whenPressed(() -> this.controlPanel.togglePiston());
    // new JoystickButton(this.operatorController, Button.kX.value).whileHeld(new ControlPanelAutoCmd(this.controlPanel, 1)); // rotation
    // new JoystickButton(this.operatorController, Button.kY.value).whileHeld(new ControlPanelAutoCmd(this.controlPanel, 2)); // position
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return this.chooser.getSelected();
  }
}
