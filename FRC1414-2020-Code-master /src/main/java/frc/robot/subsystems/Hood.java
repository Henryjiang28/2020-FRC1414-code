package frc.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.RollingAverage;

public class Hood extends SubsystemBase {
  private final CANSparkMax hoodMotor = new CANSparkMax(Constants.HOOD_MOTOR_ID, MotorType.kBrushless);
  // private final CANSparkMax hoodMotor2 = new CANSparkMax(Constants.CLIMB_ARM_MOTOR_ID, MotorType.kBrushless);
  private CANEncoder hoodEncoder;
  private CANPIDController pidController;

  private final double maxEncoderTicks = 2.42;
  private double encoderDegree = 0; // TODO

  private double minAngle = 13;//0;
  private double maxAngle = 59;//2.5;

  private double previousError = 0.0;
  private double accumulatedError = 0.0;

  private RollingAverage avg = new RollingAverage();

  private final double height = Constants.TARGET_HEIGHT - Constants.LIMELIGHT_HEIGHT;

  public Hood() {
    this.hoodEncoder = this.hoodMotor.getEncoder();
    this.pidController = this.hoodMotor.getPIDController();
    this.previousError = 0.0;
    this.accumulatedError = 0.0;
    this.resetEncoder();
    this.hoodMotor.setInverted(true);
    // this.hoodMotor2.setInverted(false);
    this.hoodMotor.setIdleMode(IdleMode.kBrake);
    // this.hoodMotor2.setIdleMode(IdleMode.kBrake);
    this.encoderDegree = (minAngle - maxAngle)/maxEncoderTicks;

    // this.hoodMotor2.follow(this.hoodMotor);

    // this.hoodEncoder.setInverted(true)

    this.pidController.setP(Constants.HOOD_MOTOR_kP);
    this.pidController.setI(Constants.HOOD_MOTOR_kI);
    this.pidController.setD(Constants.HOOD_MOTOR_kD);
    this.pidController.setIZone(Constants.HOOD_MOTOR_kIZ);
    this.pidController.setFF(Constants.HOOD_MOTOR_kFF);
    this.pidController.setOutputRange(-Constants.HOOD_MOTOR_OUTPUT, Constants.HOOD_MOTOR_OUTPUT);
  }

  public double getEncoder() {
    return this.hoodEncoder.getPosition();
  }

  public double getK() {
    return 0.5 * Constants.AIR_DENSITY * Math.PI * Constants.BALL_RADIUS_METERS * Constants.BALL_RADIUS_METERS * Constants.BALL_CX;
  }

  public double getInitialSpeed() {
    return Double.valueOf((Constants.SHOOTER_RPM) / 30 * Math.PI * Constants.SHOOTER_WHEEL_RADIUS);
  }

  public double getDelta(double distance) {
    return ( distance/height)*(distance/height) - Double.valueOf(2) * Constants.GRAVITY * distance * distance / (this.getInitialSpeed() * this.getInitialSpeed() * height) * (Double.valueOf(1) + this.getK() * distance / (Constants.BALL_MASS * getInitialSpeed())) * (Double.valueOf(1) + Constants.GRAVITY * distance * distance / (Double.valueOf(2) * height * getInitialSpeed() * getInitialSpeed()) + Constants.GRAVITY * getK() * distance * distance * distance / (Double.valueOf(3) * Constants.BALL_MASS * height * getInitialSpeed() * getInitialSpeed() * getInitialSpeed()));
  }

  public double getOptimalAngle(double distance) {
    return Math.toDegrees(Math.atan((distance / height - Math.sqrt(getDelta(distance))) / (Constants.GRAVITY * distance * distance / (this.getInitialSpeed() * this.getInitialSpeed() * height) * (1 + this.getK() * distance / (Constants.BALL_MASS * this.getInitialSpeed()))))) + (distance - 4.8) * 5;
  }


  public void moveHood(double throttle) {
    SmartDashboard.putNumber("throttle", throttle);
    if (this.maxAngle > this.getAngle() && this.getAngle() > this.minAngle) {
      this.hoodMotor.set(throttle);
    } else if (this.maxAngle <= this.getAngle()) {
      if (throttle < 0) {
        this.hoodMotor.set(0);
      } else {
        this.hoodMotor.set(throttle);
      }
    } else if (this.minAngle >= this.getAngle()) {
      if (throttle < 0) {
        this.hoodMotor.set(throttle);
      } else {
        this.hoodMotor.set(0);
      }
    }
  }

  public int detectsTarget() {
    NetworkTableInstance.getDefault().startClientTeam(1414);
    NetworkTableInstance.getDefault().startDSClient();
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    NetworkTableEntry tv = table.getEntry("tv");

    return (int)tv.getDouble(0.0);
  }

  public double getAngle() {
    return this.getEncoder() * this.encoderDegree + this.maxAngle;
  }

  public void setAngle(double angle) {
    if (angle > this.maxAngle) {
      angle = this.maxAngle;
    } else if (angle < this.minAngle) {
      angle = this.minAngle;
    }
    double encoderTicks = -(angle - this.maxAngle)*(this.maxEncoderTicks/(this.maxAngle - this.minAngle));
    SmartDashboard.putNumber("Setpoint", encoderTicks);
    this.pidController.setReference(encoderTicks, ControlType.kPosition);
  }

  public void resetAngle() {
    if (this.getAngle() < this.maxAngle) {
      this.setAngle(this.maxAngle);
    } else {
      this.resetEncoder();
    }
  }

  public double calculateVisionAngle() {
    NetworkTableInstance.getDefault().startClientTeam(1414);
    NetworkTableInstance.getDefault().startDSClient();
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    NetworkTableEntry ty = table.getEntry("ty");

    avg.add(ty.getDouble(0.0));

    return avg.getAverage();
  }

  public double calculateDistance() {
    // return 5;
    // d = (h2-h1) / tan(a1+a2)
    // return height / Math.tan(Math.toRadians(calculateVisionAngle() + Constants.LIMELIGHT_Y_ANGLE));
    return 7;
  }

  public void visionTargeting() {
    // if (this.detectsTarget() == 1) {
    //   this.setAngle(getOptimalAngle(calculateDistance()));
    // }
    this.setAngle(this.getOptimalAngle(this.calculateDistance()));
  }

  public void lowGoal() {
    this.setAngle(this.minAngle);
  }

  public void resetEncoder() {
    this.hoodEncoder.setPosition(0);
  }

  public void maxEncoder() {
    this.hoodEncoder.setPosition(this.maxEncoderTicks);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Hood Encoder", this.getEncoder());
    SmartDashboard.putNumber("Hood Angle", this.getAngle());
    SmartDashboard.putNumber("Hood Speed", this.hoodMotor.get());
    SmartDashboard.putNumber("Vision Angle", calculateVisionAngle());
    SmartDashboard.putNumber("Distance", calculateDistance());
    SmartDashboard.putNumber("Optimal Angle", getOptimalAngle(calculateDistance()));
  }
}