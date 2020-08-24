package frc.robot.commands.Indexer;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Indexer;

public class IndexerTeleopCmd extends CommandBase {

  private final Indexer indexer;
  private final DoubleSupplier throttle;
  private final BooleanSupplier reverse;
  private final BooleanSupplier aiming;
  private final DoubleSupplier shooterRPM;
  private final BooleanSupplier lowGoal;
  private final DoubleSupplier shooterThrottle;
  private boolean shooting = false;

  public IndexerTeleopCmd(Indexer indexer, DoubleSupplier throttle, DoubleSupplier shooterRPM, BooleanSupplier reverse, BooleanSupplier aiming, BooleanSupplier lowGoal, DoubleSupplier shooterThrottle) {
    this.indexer = indexer;
    this.throttle = throttle;
    this.reverse = reverse;
    this.shooterRPM = shooterRPM;
    this.aiming = aiming;
    this.lowGoal = lowGoal;
    this.shooterThrottle = shooterThrottle;
    addRequirements(this.indexer);
  }

  public void initialize() {
    this.shooting = false;
  }

  public void execute() {
    double rpm = (double)Constants.SHOOTER_RPM;

    if (this.lowGoal.getAsBoolean()) {
      rpm = Constants.LOW_GOAL_RPM;
    }

    if (this.throttle.getAsDouble() > 0.5) {
      this.indexer.blockShooter();
      if (this.reverse.getAsBoolean()) {
        this.indexer.reverseIndexer();
      } else {
        this.indexer.shiftIndexer();
        if (this.aiming.getAsBoolean()) {
          this.indexer.loadShooter();
        }
      }
    } else if ((this.shooterRPM.getAsDouble() > rpm - 50 || this.shooting) && (this.aiming.getAsBoolean() || this.lowGoal.getAsBoolean())) {//&& this.indexer.detectsTarget() == 1) {
      this.indexer.shootIndexer();
      this.indexer.loadShooter();
    }
    else {
      this.indexer.stopIndexer();
      this.indexer.stopLoader();
    }

    if (!this.shooting && this.shooterRPM.getAsDouble() > rpm - 50) {
      this.shooting = true;
    } else if (this.shooting && this.shooterThrottle.getAsDouble() < 0.5) {
      this.shooting = false;
    }
  }

  @Override
  public boolean isFinished() {
      return false;
  }

  @Override
  public void end(final boolean interrupted) {
    this.indexer.stopIndexer();
  }
}