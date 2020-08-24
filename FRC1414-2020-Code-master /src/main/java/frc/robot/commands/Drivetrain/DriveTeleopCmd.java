package frc.robot.commands.Drivetrain;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class DriveTeleopCmd extends CommandBase {

  private final Drivetrain drivetrain;
  private final DoubleSupplier throttle;
  private final DoubleSupplier turn;
  private final BooleanSupplier quickTurn;

  public DriveTeleopCmd(Drivetrain drivetrain, DoubleSupplier throttle, DoubleSupplier turn, BooleanSupplier quickTurn) {
    this.drivetrain = drivetrain;
    this.throttle = throttle;
    this.turn = turn;
    this.quickTurn = quickTurn;
    addRequirements(this.drivetrain);
  }

  public void initialize() {
  }

  public void execute() {
    this.drivetrain.driveCurvature(this.throttle.getAsDouble(), this.turn.getAsDouble(), this.quickTurn.getAsBoolean());
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    this.drivetrain.drive(0.0, 0.0);
  }
}