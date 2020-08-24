package frc.robot.commands.Hood;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Hood;

public class HoodTeleopCmd extends CommandBase {

  private final Hood hood;
  private final DoubleSupplier throttle;
  private final BooleanSupplier manual;

  public HoodTeleopCmd(Hood hood, DoubleSupplier throttle, BooleanSupplier manual){
    this.hood = hood;
    this.throttle = throttle;
    this.manual = manual;
    addRequirements(this.hood);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    if (this.manual.getAsBoolean()) {
      this.hood.moveHood(this.throttle.getAsDouble());
    } else {
      this.hood.resetAngle();
    }
  }

  @Override
  public void end(boolean interrupted) {
      this.hood.moveHood(0.0);
  }
}