/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.Rev2mDistanceSensor;
import com.revrobotics.Rev2mDistanceSensor.Port;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Indexer extends SubsystemBase {
  private final TalonSRX topBeltMotor = new TalonSRX(Constants.TOP_BELT_MOTOR_ID);
  private final TalonSRX bottomBeltMotor = new TalonSRX(Constants.BOTTOM_BELT_MOTOR_ID);
  private final TalonSRX loadingMotor = new TalonSRX(Constants.LOADING_MOTOR_ID);
  // private final Rev2mDistanceSensor distanceSensor = new Rev2mDistanceSensor(Port.kMXP);
  private double startTime;

  public Indexer() {
    this.loadingMotor.setInverted(true);
    this.bottomBeltMotor.setInverted(true);
    this.topBeltMotor.setInverted(false);
    this.bottomBeltMotor.configContinuousCurrentLimit(25);
    this.topBeltMotor.configContinuousCurrentLimit(25);
    this.bottomBeltMotor.enableCurrentLimit(true);
    this.topBeltMotor.enableCurrentLimit(true);
    this.bottomBeltMotor.setNeutralMode(NeutralMode.Brake);
    this.topBeltMotor.setNeutralMode(NeutralMode.Brake);
    startTime = Timer.getFPGATimestamp();
  }

  public void loadShooter() {
    this.loadingMotor.set(ControlMode.PercentOutput, Constants.LOADING_MOTOR_SPEED);
  }

  public void blockShooter() {
    this.loadingMotor.set(ControlMode.PercentOutput, 0.0);
  }

  public void stopLoader() {
    this.loadingMotor.set(ControlMode.PercentOutput, 0.0);
  }

  public void shiftIndexer() {
    this.topBeltMotor.set(ControlMode.PercentOutput, Constants.BELT_MOTOR_SPEED);
    this.bottomBeltMotor.set(ControlMode.PercentOutput, Constants.BELT_MOTOR_SPEED);
  }

  public void shootIndexer() {
    this.topBeltMotor.set(ControlMode.PercentOutput, Constants.BELT_SHOOTING_SPEED);
    this.bottomBeltMotor.set(ControlMode.PercentOutput, Constants.BELT_SHOOTING_SPEED);
  }

  public void alternateIndexer() {
    double current = Timer.getFPGATimestamp();
    SmartDashboard.putNumber("Elapsed TIme", current - startTime);
    if ((current - startTime) % 1.0 > 0.25) {
      this.shiftIndexer();
    } else {
      this.reverseIndexer();
    }
  }

  public void reverseIndexer() {
    this.topBeltMotor.set(ControlMode.PercentOutput, -Constants.BELT_MOTOR_SPEED);
    this.bottomBeltMotor.set(ControlMode.PercentOutput, -Constants.BELT_MOTOR_SPEED);
  }

  public void stopIndexer() {
    this.topBeltMotor.set(ControlMode.PercentOutput, 0.0);
    this.bottomBeltMotor.set(ControlMode.PercentOutput, 0.0);
  }

  public int detectsTarget() {
    NetworkTableInstance.getDefault().startClientTeam(1414);
    NetworkTableInstance.getDefault().startDSClient();
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    NetworkTableEntry tv = table.getEntry("tv");

    return (int)tv.getDouble(0.0);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Loading Speed", this.loadingMotor.getMotorOutputPercent());
    SmartDashboard.putNumber("Top Belt Speed", this.topBeltMotor.getMotorOutputPercent());
    SmartDashboard.putNumber("Bottom Belt Speed", this.bottomBeltMotor.getMotorOutputPercent());
  }
}
