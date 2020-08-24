package frc.robot.commands.Hood;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Hood;

public class HoodAutoCmd extends CommandBase {

  private final Hood hood;
  private final DoubleSupplier shooterRPM;

  public HoodAutoCmd(Hood hood, DoubleSupplier shooterRPM){
    this.hood = hood;
    this.shooterRPM = shooterRPM;
    addRequirements(this.hood);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    // if (this.shooterRPM.getAsDouble() > Constants.SHOOTER_RPM - 200) {
    this.hood.visionTargeting();
    // }
  }

  @Override
  public void end(boolean interrupted) {
    this.hood.moveHood(0.0);
  }
}