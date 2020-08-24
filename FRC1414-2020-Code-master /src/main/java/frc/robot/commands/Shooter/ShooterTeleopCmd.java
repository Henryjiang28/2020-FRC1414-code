package frc.robot.commands.Shooter;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;

public class ShooterTeleopCmd extends CommandBase {
  private final Shooter shooter;
  private final DoubleSupplier throttle;
  private final BooleanSupplier lowGoal;

  public ShooterTeleopCmd(Shooter shooter, DoubleSupplier throttle, BooleanSupplier lowGoal) {
    this.shooter = shooter;
    this.throttle = throttle;
    this.lowGoal = lowGoal;
    addRequirements(this.shooter);
  }

  public void initialize() {
  }

  public void execute() {
    if (this.throttle.getAsDouble() > 0.5) {
      if (this.lowGoal.getAsBoolean()) {
        this.shooter.lowGoal();
      } else {
        this.shooter.shoot();
      }
    } else {
      this.shooter.stop();
    }
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    this.shooter.stop();
  }
}