/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import java.io.File;
import java.io.IOException;

import com.kauailabs.navx.frc.AHRS;
import frc.robot.Constants;
import frc.robot.util.CurvatureUtils;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.networktables.NetworkTable;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.Waypoint;
import jaci.pathfinder.followers.EncoderFollower;
import jaci.pathfinder.modifiers.TankModifier;

public class Drivetrain extends SubsystemBase {
  private final TalonFX leftMotor = new TalonFX(Constants.LEFT_DRIVE_MOTOR_ID);
  private final TalonFX leftMotor2 = new TalonFX(Constants.LEFT_DRIVE_MOTOR2_ID);
  private final TalonFX rightMotor = new TalonFX(Constants.RIGHT_DRIVE_MOTOR_ID);
  private final TalonFX rightMotor2 = new TalonFX(Constants.RIGHT_DRIVE_MOTOR2_ID);

  public static double trackWidth = 0.0; // distance between the wheels in meters
  public static double gearRatio = 0.0;  // gearRatio of the robot
  public static double wheelRadius = 0.0;
  public final double ks = 0.0;
  public final double kv = 0.0;
  public final double ka = 0.0;

  AHRS gyro = new AHRS(I2C.Port.kMXP);

  public Drivetrain() {
    this.leftMotor2.follow(this.leftMotor);
    this.rightMotor2.follow(this.rightMotor);

    this.leftMotor.setInverted(false);
    this.rightMotor.setInverted(true);
    this.leftMotor2.setInverted(false);
    this.rightMotor2.setInverted(true);
    
    this.leftMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 0);
    this.rightMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 0);

    this.leftMotor.setNeutralMode(NeutralMode.Brake);
    this.rightMotor.setNeutralMode(NeutralMode.Brake);
    this.leftMotor2.setNeutralMode(NeutralMode.Brake);
    this.rightMotor2.setNeutralMode(NeutralMode.Brake);

    resetGyro();
    resetEncoders();
    resetErrors();
  }

  public void resetGyro() {
    this.gyro.reset();
  }

  public void arcadeDrive(double throttle, double turn) {
    this.leftMotor.set(ControlMode.PercentOutput, (throttle + turn) * this.maxOutput);
    this.rightMotor.set(ControlMode.PercentOutput, (throttle - turn) * this.maxOutput);
  }

  public void drive(double powLeft, double powRight) {
    this.leftMotor.set(ControlMode.PercentOutput, powLeft * this.maxOutput);
    this.rightMotor.set(ControlMode.PercentOutput, powRight * this.maxOutput);
  }

  public void resetEncoders() {
    this.leftMotor.setSelectedSensorPosition(0, 0, 0);
    this.rightMotor.setSelectedSensorPosition(0, 0, 0);
  }

  public void setMaxOutput(double max) {
    this.maxOutput = max;
  }

  double deadband = 0.02;
  double maxOutput = 1.0;
  double quickStopThreshold = 0.2;
  double quickStopAlpha = 0.1;
  double quickStopAccumulator = 0.0;
  double lastHeadingError = 0.0;
  double errorAccumulated = 0.0;

  public void driveCurvature(double throttle, double turn, boolean quick) {
    throttle = CurvatureUtils.limit(throttle);
    throttle = CurvatureUtils.applyDeadband(throttle, this.deadband);

    turn = CurvatureUtils.limit(turn);
    turn = CurvatureUtils.applyDeadband(turn, this.deadband);

    double angularPower;
    boolean overPower;

    if (quick) {
      if (Math.abs(throttle) < this.quickStopThreshold) {
        this.quickStopAccumulator = (1 - this.quickStopAlpha) * this.quickStopAccumulator
            + this.quickStopAlpha * CurvatureUtils.limit(turn) * 2;
      }
      overPower = true;
      angularPower = turn;
    } else {
      overPower = false;
      angularPower = Math.abs(throttle) * turn - this.quickStopAccumulator;

      if (this.quickStopAccumulator > 1) {
        this.quickStopAccumulator -= 1;
      } else if (this.quickStopAccumulator < -1) {
        this.quickStopAccumulator += 1;
      } else {
        this.quickStopAccumulator = 0.0;
      }
    }

    double leftMotorOutput = throttle + angularPower;
    double rightMotorOutput = throttle - angularPower;

    // If rotation is overpowered, reduce both outputs to within acceptable range
    if (overPower) {
      if (leftMotorOutput > 1.0) {
        rightMotorOutput -= leftMotorOutput - 1.0;
        leftMotorOutput = 1.0;
      } else if (rightMotorOutput > 1.0) {
        leftMotorOutput -= rightMotorOutput - 1.0;
        rightMotorOutput = 1.0;
      } else if (leftMotorOutput < -1.0) {
        rightMotorOutput -= leftMotorOutput + 1.0;
        leftMotorOutput = -1.0;
      } else if (rightMotorOutput < -1.0) {
        leftMotorOutput -= rightMotorOutput + 1.0;
        rightMotorOutput = -1.0;
      }
    }

    // Normalize the wheel speed
    double maxMagnitude = Math.max(Math.abs(leftMotorOutput), Math.abs(rightMotorOutput));
    if (maxMagnitude > 1.0) {
      leftMotorOutput /= maxMagnitude;
      rightMotorOutput /= maxMagnitude;
    }

    leftMotor.set(ControlMode.PercentOutput, leftMotorOutput * this.maxOutput);// need a maxouput?
    rightMotor.set(ControlMode.PercentOutput, rightMotorOutput * this.maxOutput);
  }

  // get encoder value
  public int getEncoderRawLeft() {
    return leftMotor.getSelectedSensorPosition(0);
  }

  public int getEncoderRawRight() {
    return rightMotor.getSelectedSensorPosition(0);
  }

  public double getGyroAngle() {
    return gyro.getAngle();
  }

  public double getEncoderDistanceMetersRight() {
    return (getEncoderRawRight() * Math.PI * DrivetrainProfiling.wheel_diameter) / DrivetrainProfiling.ticks_per_rev / DrivetrainProfiling.gearRatio;
  }

  public double getEncoderDistanceMetersLeft() {
    return (getEncoderRawLeft() * Math.PI * DrivetrainProfiling.wheel_diameter) / DrivetrainProfiling.ticks_per_rev / DrivetrainProfiling.gearRatio;
  }

  public void driveStraight(double throttle, double angle) {
    double error = angle - this.getGyroAngle();
    this.errorAccumulated += error * Constants.TIME_STEP;
    double turn = (DrivetrainProfiling.kp * error)
      + (DrivetrainProfiling.kd * (this.getGyroAngle() - this.lastHeadingError));
    this.lastHeadingError = error;
    this.leftMotor.set(ControlMode.PercentOutput, (throttle - turn) * this.maxOutput);
    this.rightMotor.set(ControlMode.PercentOutput, (throttle + turn) * this.maxOutput);
  }

  public void turnToAngle(double angle) {
    double error = angle - this.getGyroAngle();
    this.errorAccumulated += error * Constants.TIME_STEP;
    double turn = (DrivetrainProfiling.kp * error) + (DrivetrainProfiling.ki * this.errorAccumulated) + (DrivetrainProfiling.kd * (error - this.lastHeadingError));
    this.lastHeadingError = error;
    this.leftMotor.set(ControlMode.PercentOutput, turn * 0.5);
    this.rightMotor.set(ControlMode.PercentOutput, -turn * 0.5);
  }

  public void resetErrors() {
    this.lastHeadingError = 0.0;
    this.errorAccumulated = 0.0;
  }

  public double generateHashCode(final Waypoint[] path) {
    double hash = 1.0;
    for (int i = 0; i < path.length; i++) {
      hash = ((path[i].x * 3) + (path[i].y * 7) + (path[i].angle * 11));
    }
    return (int) Math.abs(hash * 1000) * path.length;
  }

  public EncoderFollower[] pathSetup(Waypoint[] path) {
    EncoderFollower left = new EncoderFollower();
    EncoderFollower right = new EncoderFollower();

    Trajectory.Config config = new Trajectory.Config(Trajectory.FitMethod.HERMITE_CUBIC, Trajectory.Config.SAMPLES_HIGH,
        Drivetrain.DrivetrainProfiling.dt, Drivetrain.DrivetrainProfiling.max_velocity,
        Drivetrain.DrivetrainProfiling.max_acceleration, Drivetrain.DrivetrainProfiling.max_jerk);

    String pathHash = String.valueOf(generateHashCode(path));
    SmartDashboard.putString("Path Hash", pathHash);

    Trajectory toFollow;

    SmartDashboard.putNumber("HASH CREATED", 0);

    File trajectory = new File("/home/lvuser/" + pathHash + ".csv");
    if (!trajectory.exists()) {
      SmartDashboard.putNumber("GENERATING COMMAND", 0);
      toFollow = Pathfinder.generate(path, config);
      SmartDashboard.putNumber("GENERATED COMMAND", 0);
      Pathfinder.writeToCSV(trajectory, toFollow);
      SmartDashboard.putNumber(pathHash + ".csv not found, wrote to file", 0);
    } else {
      SmartDashboard.putNumber("READING COMMAND", 0);
      SmartDashboard.putNumber(pathHash + ".csv read from file", 0);
      try {
        toFollow = Pathfinder.readFromCSV(trajectory);
      } catch (IOException e) {
        toFollow = new Trajectory(0);
        e.printStackTrace();
      }
    }

    SmartDashboard.putNumber("DONE CREATING COMMAND", 0);

    TankModifier modifier = new TankModifier(toFollow).modify(Drivetrain.DrivetrainProfiling.wheel_base_width);
    DrivetrainProfiling.last_gyro_error = 0.0;

    left = new EncoderFollower(modifier.getLeftTrajectory());
    right = new EncoderFollower(modifier.getRightTrajectory());

    left.configureEncoder(getEncoderRawLeft(), (int)(DrivetrainProfiling.ticks_per_rev/DrivetrainProfiling.gearRatio), DrivetrainProfiling.wheel_diameter);
    right.configureEncoder(getEncoderRawRight(), (int)(DrivetrainProfiling.ticks_per_rev/DrivetrainProfiling.gearRatio), DrivetrainProfiling.wheel_diameter);
    left.configurePIDVA(DrivetrainProfiling.kp, DrivetrainProfiling.ki, DrivetrainProfiling.kd, DrivetrainProfiling.kv,
        DrivetrainProfiling.ka);
    right.configurePIDVA(DrivetrainProfiling.kp, DrivetrainProfiling.ki, DrivetrainProfiling.kd, DrivetrainProfiling.kv,
        DrivetrainProfiling.ka);

    return new EncoderFollower[] { 
      left, // 0
      right, // 1
    };
  }

  public void resetForPath() {
    isProfileFinished = false;
    resetEncoders();
    resetGyro();
    resetErrors();
  }

  public void resetPathAngleOffset() {
    DrivetrainProfiling.path_angle_offset = 0.0;
  }

  private boolean isProfileFinished = false;

  public boolean getIsProfileFinished() {
    return isProfileFinished;
  }

  /*
   * If going !reverse going forward x is positive, going left y is positive,
   * turning left is positive If going reverse going backwards x is positive,
   * going right y is negative, turning left is negative
   */
  public void pathFollow(final EncoderFollower[] followers, final boolean reverse) {
    final EncoderFollower left = followers[0];
    final EncoderFollower right = followers[1];
    double l = 0.0;
    double r = 0.0;
    double localGp = DrivetrainProfiling.kp;
    if (!reverse) {
      localGp *= -1;
      l = left.calculate(getEncoderRawLeft());
      r = right.calculate(getEncoderRawRight());
    } else {
      r = right.calculate(-getEncoderRawRight());
      l = left.calculate(-getEncoderRawLeft());
    }

    final double gyro_heading = reverse ? -getGyroAngle() - DrivetrainProfiling.path_angle_offset
        : getGyroAngle() + DrivetrainProfiling.path_angle_offset;
    final double angle_setpoint = Pathfinder.r2d(left.getHeading());
    SmartDashboard.putNumber("Angle setpoint", angle_setpoint);
    final double angleDifference = Pathfinder.boundHalfDegrees(angle_setpoint - gyro_heading);
    SmartDashboard.putNumber("Angle difference", angleDifference);

    double turnMultiplier = 0.8;

    if (reverse) {
      turnMultiplier = -0.8;
    } else {
      turnMultiplier = 0.8;
    }

    final double turn = turnMultiplier * (-1.0 / 85) * angleDifference;
    SmartDashboard.putNumber("turn", turn);

    // double turn = localGp * angleDifference + (DrivetrainProfiling.gd *
    // ((angleDifference - DrivetrainProfiling.last_gyro_error) /
    // DrivetrainProfiling.dt));

    DrivetrainProfiling.last_gyro_error = angleDifference;

    if (!reverse) {
      drive(l + turn, r - turn);
    } else {
      drive(-l + turn, -r - turn);
    }
    SmartDashboard.putNumber("l", l);
    SmartDashboard.putNumber("r", r);

    if (left.isFinished() && right.isFinished()) {
      isProfileFinished = true;
      DrivetrainProfiling.path_angle_offset = 0.0;
    }
  }

  public static class DrivetrainProfiling {
    public static double kp = 0.0375; // 0.0375
    public static double ki = 0.0; // 0.0
    public static double kd = 0.0; // 0.0

    // Gyro Logging for Motion Profiling
    public static double last_gyro_error = 0.0;

    public static final double gearRatio = 7.92;
    public static double path_angle_offset = 0.0;
    public static final double max_velocity = 1.5; // 2.58 m/s Actual
    public static final double kv = 1.0 / max_velocity;
    public static final double max_acceleration = 0.8; // Estimated # 3.8
    public static final double ka = 0.015; // 0.015
    public static final double max_jerk = 0.5; // 16.0
    public static final double wheel_diameter = 0.15; // 0.1016
    public static final double wheel_base_width = 0.66; // 0.66
    public static final int ticks_per_rev = 2048; // Quad Encoder
    public static final double dt = 0.02; // Calculated - Confirmed
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Left Drive Output", this.leftMotor.getMotorOutputPercent());
    SmartDashboard.putNumber("Right Drive Output", this.rightMotor.getMotorOutputPercent());

    SmartDashboard.putNumber("Left Encoder", this.getEncoderRawLeft());
    SmartDashboard.putNumber("Right Encoder", this.getEncoderRawRight());

    SmartDashboard.putNumber("Left Distance", this.getEncoderDistanceMetersLeft());
    SmartDashboard.putNumber("Right Distance", this.getEncoderDistanceMetersRight());

    SmartDashboard.putNumber("Gyro Angle", this.getGyroAngle());
  }
}
