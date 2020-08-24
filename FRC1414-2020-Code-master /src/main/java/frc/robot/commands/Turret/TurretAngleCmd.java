package frc.robot.commands.Turret;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Turret;

public class TurretAngleCmd extends CommandBase {

  private final Turret m_turret;

  public TurretAngleCmd(Turret turret) {
    m_turret = turret;
    addRequirements(turret);
  }

  public void initialize() {
    // enable brakemode
  }

  // regulate percentage of speed output
  public void execute(double angle) {

    m_turret.setAngle(angle);

  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {

  }
}