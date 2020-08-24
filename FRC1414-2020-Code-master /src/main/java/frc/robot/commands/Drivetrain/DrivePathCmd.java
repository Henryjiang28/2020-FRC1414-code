package frc.robot.commands.Drivetrain;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import jaci.pathfinder.Waypoint;
import jaci.pathfinder.followers.EncoderFollower;

public class DrivePathCmd extends CommandBase {

  private final Drivetrain drivetrain;
  
  private final Waypoint[] path;
  private final EncoderFollower[] followers;

  public DrivePathCmd(Drivetrain drivetrain, Waypoint[] path) {
    this.drivetrain = drivetrain;
    this.path = path;

    System.out.println("PATH SETUP COMMAND");
    this.followers = this.drivetrain.pathSetup(path);
    System.out.println("DONE PATH SETUP COMMAND");

    addRequirements(this.drivetrain);
  }

  public void initialize() {
    System.out.println("RUNNING INIT COMMAND");
    this.drivetrain.resetForPath();
    this.drivetrain.pathFollow(this.followers, false);
    System.out.println("DONE INIT COMMAND");
  }

  public void execute() {
    this.drivetrain.pathFollow(followers, false);
  }

  @Override
  public boolean isFinished() {
    return this.drivetrain.getIsProfileFinished();
  }

  @Override
  public void end(boolean interrupted) {
    this.drivetrain.drive(0, 0);
  }
}