/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
  private final TalonSRX intakeMotor = new TalonSRX(Constants.INTAKE_MOTOR_ID);
  private final DoubleSolenoid intakePiston = new DoubleSolenoid(Constants.PCM_ID,
  Constants.INTAKE_PISTON_FORWARD, Constants.INTAKE_PISTON_REVERSE);
  private Value pistonState = Constants.INTAKE_PISTON_CLOSED;
  private final Compressor compressor = new Compressor(Constants.PCM_ID);

  public Intake() {
    this.setReverse();
  }

  public Value getPistonState() {
    return this.pistonState;
  }

  public void toggleIntake() {
    if (getPistonState() == Constants.INTAKE_PISTON_OPEN) {
      this.intakePiston.set(Constants.INTAKE_PISTON_CLOSED);
      this.pistonState = Constants.INTAKE_PISTON_CLOSED;
    } else if (getPistonState() == Constants.INTAKE_PISTON_CLOSED) {
      this.intakePiston.set(Constants.INTAKE_PISTON_OPEN);
      this.pistonState = Constants.INTAKE_PISTON_OPEN;
    }
  }

  public void setForward() {
    this.intakePiston.set(Constants.INTAKE_PISTON_OPEN);
    this.pistonState = Constants.INTAKE_PISTON_OPEN;
    this.compressor.stop();
  }

  public void setReverse() {
    this.intakePiston.set(Constants.INTAKE_PISTON_CLOSED);
    this.pistonState = Constants.INTAKE_PISTON_CLOSED;
    this.compressor.start();
  }

  public void intake() {
    this.intakeMotor.set(ControlMode.PercentOutput, Constants.INTAKE_MOTOR_SPEED);
  }


  public void outtake() {
    this.intakeMotor.set(ControlMode.PercentOutput, Constants.OUTTAKE_MOTOR_SPEED);
  }

  public void stopIntake() {
    this.intakeMotor.set(ControlMode.PercentOutput, 0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Intake Speed", this.intakeMotor.getMotorOutputPercent());

    if (this.getPistonState() == Constants.INTAKE_PISTON_OPEN) {
      SmartDashboard.putBoolean("Intake Deployed", true);
    } else {
      SmartDashboard.putBoolean("Intake Deployed", false);
    }
  }
}
