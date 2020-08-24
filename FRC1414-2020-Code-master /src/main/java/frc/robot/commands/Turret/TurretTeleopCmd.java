package frc.robot.commands.Turret;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Turret;

public class TurretTeleopCmd extends CommandBase {

  private final Turret turret;
  private final DoubleSupplier turn;
  private final BooleanSupplier manual;

  public TurretTeleopCmd(Turret turret, DoubleSupplier turn, BooleanSupplier manual) {
    this.turret = turret;
    this.turn = turn;
    this.manual = manual;
    addRequirements(this.turret);
  }

  public void initialize() {
  }

  public void execute() {
    // if (!this.turret.getHomed()) {
    //   this.turret.home();
    // } else {
    if (this.manual.getAsBoolean()) {
      this.turret.setVisionMode(false);
      this.turret.moveTurret(this.turn.getAsDouble());
    } else {
      this.turret.moveTurret(0.0);
    }
    // }
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    this.turret.moveTurret(0.0);
  }
}