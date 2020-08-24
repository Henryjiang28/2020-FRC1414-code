package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Intake;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;


public class IntakeTeleopCmd extends CommandBase {

  private final Intake intake;
  private final DoubleSupplier throttle;
  private final BooleanSupplier reverse;

  public IntakeTeleopCmd(Intake intake, DoubleSupplier throttle, BooleanSupplier reverse) {
    this.intake = intake;
    this.throttle = throttle;
    this.reverse = reverse;
    addRequirements(this.intake);
  }

  public void initialize() {
  }

  public void execute() {
    if ((this.intake.getPistonState() == Constants.INTAKE_PISTON_OPEN) && (this.throttle.getAsDouble() > 0.5)) {
      if (this.reverse.getAsBoolean()) {
        this.intake.outtake();
      } else {
        this.intake.intake();
      }
    } else {
      this.intake.stopIntake();
    }
  }

  @Override
  public boolean isFinished() {
      return false;
  }

  @Override
  public void end(boolean interrupted) {
    this.intake.stopIntake();
  }
}