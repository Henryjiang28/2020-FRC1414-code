package frc.robot.commands.Drivetrain;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class DriveAutoCmd extends CommandBase {

  private final Drivetrain drivetrain;
  private final double distance;

  public DriveAutoCmd(Drivetrain drivetrain, double distance) {
    this.drivetrain = drivetrain;
    this.distance = distance;

    addRequirements(this.drivetrain);
  }

  public void initialize() {
    this.drivetrain.resetEncoders();
    this.drivetrain.resetGyro();
    this.drivetrain.resetErrors();
  }

  public void execute() {
    if (this.drivetrain.getEncoderDistanceMetersLeft() > this.distance + 0.2) {
      this.drivetrain.driveStraight(-0.3, this.drivetrain.getGyroAngle());
    } else if (this.drivetrain.getEncoderDistanceMetersLeft() < this.distance - 0.2) {
      this.drivetrain.driveStraight(0.3, this.drivetrain.getGyroAngle());
    } else {
      end(false);
    }
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    this.drivetrain.drive(0, 0);
  }
}