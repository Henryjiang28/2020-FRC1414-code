package frc.robot.commands.ControlPanel;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ControlPanel;
import frc.robot.subsystems.Intake;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;


public class ControlPanelAutoCmd extends CommandBase {

  private final ControlPanel controlPanel;
  private final int mode;

  public ControlPanelAutoCmd(ControlPanel controlPanel, int mode) {
    this.controlPanel = controlPanel;
    this.mode = mode;
    addRequirements(this.controlPanel);
  }

  public void initialize() {
  }

  public void execute() {
    if (this.mode == 1) {
      this.controlPanel.rotationControl();
    } else if (this.mode == 2) {
      this.controlPanel.positionControl();
    } else {
      this.controlPanel.stopMotor();
    }
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