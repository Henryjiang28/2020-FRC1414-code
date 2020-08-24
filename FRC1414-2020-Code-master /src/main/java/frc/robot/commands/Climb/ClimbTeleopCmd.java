package frc.robot.commands.Climb;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climb;

public class ClimbTeleopCmd extends CommandBase {

  private final Climb climb;
  private final DoubleSupplier armThrottle;
  private final DoubleSupplier elevatorThrottle;
  private final BooleanSupplier climbing;

  public ClimbTeleopCmd(Climb climb, DoubleSupplier armThrottle, DoubleSupplier elevatorThrottle, BooleanSupplier climbing){
    this.climb = climb;
    this.armThrottle = armThrottle;
    this.elevatorThrottle = elevatorThrottle;
    this.climbing = climbing;
    addRequirements(this.climb);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    if (this.climbing.getAsBoolean()) {
        this.climb.directArm(this.armThrottle.getAsDouble());
        this.climb.directElevator(this.elevatorThrottle.getAsDouble());
    } else {
        this.climb.stopArm();
        this.climb.stopElevator();
    }
  }

  @Override
  public void end(boolean interrupted) {
      this.climb.stopArm();
      this.climb.stopElevator();
  }
}