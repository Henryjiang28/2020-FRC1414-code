package frc.robot.commands.Turret;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Turret;

public class TurretAutoCmd extends CommandBase {

  private final Turret turret;

  public TurretAutoCmd(Turret turret){
    this.turret = turret;
    addRequirements(this.turret);
  }

  @Override
  public void initialize() {
    this.turret.setVisionMode(true);
  }

  @Override
  public void execute() {
    this.turret.setVisionMode(true);
    if (this.turret.detectsTarget() == 1) {
      this.turret.visionTargeting();
    } else {
      this.turret.findTarget();
    }
  }

  @Override
  public void end(boolean interrupted) {
    this.turret.moveTurret(0.0);
    this.turret.setVisionMode(false);
  }
}