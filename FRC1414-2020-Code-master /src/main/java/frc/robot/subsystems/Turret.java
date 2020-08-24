package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.RollingAverage;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Turret extends SubsystemBase {

  // private final double maxEncoderTicks = 200000.0; // TODO
  private final double encoderDegree = (double)360/(2048*21*20); // 1.19 is for error

  private double previousError = 0.0;
  private double accumulatedError = 0.0;
  private boolean homed = false;

  private double minAngle = -120;
  private double maxAngle = 120;

  private boolean findingForward = false;

  private TalonSRX turretMotor = new TalonSRX(Constants.TURRET_MOTOR_ID);
  // private DigitalInput limitSwitch = new DigitalInput(0);

  private RollingAverage avg = new RollingAverage();

  public Turret() {
    this.turretMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 0);
    this.turretMotor.setNeutralMode(NeutralMode.Brake);
    /* Config the peak and nominal outputs, 12V means full */
		this.turretMotor.configNominalOutputForward(0, 0);
		this.turretMotor.configNominalOutputReverse(0, 0);
		this.turretMotor.configPeakOutputForward(Constants.TURRET_MOTOR_MAX_OUTPUT, 0);
    this.turretMotor.configPeakOutputReverse(-Constants.TURRET_MOTOR_MAX_OUTPUT, 0);
    
    this.turretMotor.configAllowableClosedloopError(0, 0, 0);

    this.turretMotor.config_kF(0, Constants.TURRET_MOTOR_kF, 0);
		this.turretMotor.config_kP(0, Constants.TURRET_MOTOR_kP, 0);
		this.turretMotor.config_kI(0, Constants.TURRET_MOTOR_kI, 0);
		this.turretMotor.config_kD(0, Constants.TURRET_MOTOR_kD, 0);

    this.resetEncoder();
    this.previousError = 0.0;
    this.accumulatedError = 0.0;

    this.setVisionMode(false);
  }

  // public void home() {
  //   if (this.limitSwitch.get()) {
  //     if (this.getAngle() > this.maxAngle || this.getAngle() < this.minAngle) {
  //       this.turretMotor.set(ControlMode.PercentOutput, 0.0);
  //     } else {
  //       this.turretMotor.set(ControlMode.PercentOutput, 0.5);
  //     }
  //   } else {
  //     this.turretMotor.set(ControlMode.PercentOutput, 0.0);
  //     this.resetEncoder();
  //     this.homed = true;
  //   }
  // }

  // public boolean getHomed() {
  //   return this.homed;
  // }

  public void moveTurret(double throttle) {
    if (this.maxAngle > this.getAngle() && this.getAngle() > this.minAngle) {
      this.turretMotor.set(ControlMode.PercentOutput, throttle);
    } else if (this.maxAngle <= this.getAngle()) {
      if (throttle > 0) {
        this.turretMotor.set(ControlMode.PercentOutput, 0);
      } else {
        this.turretMotor.set(ControlMode.PercentOutput, throttle);
      }
    } else if (this.minAngle >= this.getAngle()) {
      if (throttle > 0) {
        this.turretMotor.set(ControlMode.PercentOutput, throttle);
      } else {
        this.turretMotor.set(ControlMode.PercentOutput, 0);
      }
    }
  }

  public void findTarget() {
    if (findingForward) {
      this.moveTurret(0.5);
      if (this.getAngle() >= this.maxAngle) {
        this.moveTurret(0.0);
        this.findingForward = false;
      }
    } else {
      this.moveTurret(-0.5);
      if (this.getAngle() <= this.minAngle) {
        this.moveTurret(0.0);
        this.findingForward = true;
      }
    }
  }

  public double getAngle() {
    return this.getEncoder() * this.encoderDegree;
  }

  public boolean inPosition() {
    if (this.getAngle() < calculateVisionAngle() + 2 && this.getAngle() > calculateVisionAngle() - 2) {
      return true;
    } else {
      return false;
    }
  }

  public void setAngle(double angle) {
    if (angle > this.maxAngle) {
      angle = this.maxAngle;
    } else if (angle < this.minAngle) {
      angle = this.minAngle;
    }
    double encoderTicks = angle/this.encoderDegree;
    this.turretMotor.set(ControlMode.Position, encoderTicks);
  }

  public void resetAngle() {
    this.setAngle(0);
  }

  public double getEncoder() {
    return this.turretMotor.getSelectedSensorPosition(0);
  }

  public void resetEncoder() {
    this.turretMotor.setSelectedSensorPosition(0, 0, 0);
  }

  public double calculateVisionAngle() {
    NetworkTableInstance.getDefault().startClientTeam(1414);
    NetworkTableInstance.getDefault().startDSClient();
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    NetworkTableEntry tx = table.getEntry("tx");

    avg.add(this.getAngle() - tx.getDouble(0.0));

    return avg.getAverage();
  }

  public void visionTargeting() {
    this.setAngle(calculateVisionAngle());
  }

  public int detectsTarget() {
    NetworkTableInstance.getDefault().startClientTeam(1414);
    NetworkTableInstance.getDefault().startDSClient();
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    NetworkTableEntry tv = table.getEntry("tv");

    return (int)tv.getDouble(0.0);
  }

  public void lowGoal() {
    this.setAngle(0);
  }

  public void setVisionMode(boolean on) {
    NetworkTableInstance.getDefault().startClientTeam(1414);
    NetworkTableInstance.getDefault().startDSClient();
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    NetworkTableEntry camMode = table.getEntry("camMode");
    NetworkTableEntry ledMode = table.getEntry("ledMode");
    if (on) {
      camMode.setDouble(0);
      ledMode.setDouble(3);
    } else {
      camMode.setDouble(1);
      ledMode.setDouble(1);
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Turret Encoder", this.getEncoder());
    SmartDashboard.putNumber("Turret Angle", this.getAngle());
    SmartDashboard.putNumber("Turret Speed", this.turretMotor.getMotorOutputPercent());
    // SmartDashboard.putBoolean("Turret Limit Switch", !this.limitSwitch.get());
    SmartDashboard.putNumber("Turret Limelight Angle", this.calculateVisionAngle());
    SmartDashboard.putNumber("Targets Detected", this.detectsTarget());
  }
}