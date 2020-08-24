package frc.robot.commands.ControlPanel;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ControlPanel;
import frc.robot.subsystems.Intake;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;


public class ControlPanelTeleopCmd extends CommandBase {

  private final ControlPanel controlPanel;

  public ControlPanelTeleopCmd(ControlPanel controlPanel) {
    this.controlPanel = controlPanel;
    addRequirements(this.controlPanel);
  }

  public void initialize() {
  }

  public void execute() {
    this.controlPanel.stopMotor();
  }

  @Override
  public boolean isFinished() {
      return false;
  }

  @Override
  public void end(boolean interrupted) {
    this.controlPanel.stopMotor();
  }
}