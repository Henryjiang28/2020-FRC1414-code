/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final double TIME_STEP = 0.02;

  public static final int DRIVE_CONTROLLER_ID = 0;
  public static final int OPERATOR_CONTROLLER_ID = 1;
  public static final int BACKUP_JOYSTICK_ID = 2;

  public static final double DRIVE_STICK_DEADBAND = 0.1;

  public static final int PCM_ID = 30;

  public static final int LEFT_DRIVE_MOTOR_ID = 1;
  public static final int LEFT_DRIVE_MOTOR2_ID = 2;
  public static final int RIGHT_DRIVE_MOTOR_ID = 3;
  public static final int RIGHT_DRIVE_MOTOR2_ID = 4;

  public static final int TURRET_MOTOR_ID = 11;
  public static final double TURRET_MOTOR_kF = 0.0;
  public static final double TURRET_MOTOR_kP = 0.03;
  public static final double TURRET_MOTOR_kI = 0.0;
  public static final double TURRET_MOTOR_kD = 0.01;
  public static final double TURRET_MOTOR_MAX_OUTPUT = 1.0;

  public static final int HOOD_MOTOR_ID = 13;
  public static final double HOOD_MOTOR_kP = 0.5;
  public static final double HOOD_MOTOR_kI = 0.0;
  public static final double HOOD_MOTOR_kD = 0.0;
  public static final double HOOD_MOTOR_kIZ = 0.0;
  public static final double HOOD_MOTOR_kFF = 0.0;
  public static final double HOOD_MOTOR_OUTPUT = 0.7;

  public static final double INTAKE_MOTOR_SPEED = 0.6;
  public static final double OUTTAKE_MOTOR_SPEED = -0.6;

  public static final int INTAKE_MOTOR_ID = 7;
  public static final int INTAKE_PISTON_FORWARD = 2;
  public static final int INTAKE_PISTON_REVERSE = 3;

  public static final Value INTAKE_PISTON_OPEN = Value.kReverse;
  public static final Value INTAKE_PISTON_CLOSED = Value.kForward;

  public static final int TOP_BELT_MOTOR_ID = 8;
  public static final int BOTTOM_BELT_MOTOR_ID = 9;
  public static final int LOADING_MOTOR_ID = 10;
  public static final double BELT_MOTOR_SPEED = 0.4;
  public static final double BELT_SHOOTING_SPEED = 0.4;
  public static final double LOADING_MOTOR_SPEED = 0.6;

  public static final int SHOOTER_ID_1 = 5;
  public static final int SHOOTER_ID_2 = 6;
  public static final int SHOOTER_PID_LOOP_ID = 0;
  public static final double SHOOTER_kF = 0.3;
  public static final double SHOOTER_kP = 0.7; // TODO
  public static final double SHOOTER_kI = 0.0; // TODO
  public static final double SHOOTER_kD = 0.0; // TODO
  public static final int SHOOTER_TIMEOUT_MS = 0;
  public static final int SHOOTER_RPM = 5600;
  public static final int LOW_GOAL_RPM = 1000;

  public static final double GRAVITY = 9.81;
  public static final double BALL_RADIUS_METERS = 0.0889;
  public static final double AIR_DENSITY = 1.21;
  public static final double TARGET_HEIGHT = 2.29;//2.496;
  public static final double BALL_CX = 0.7;
  public static final double BALL_MASS = 0.136;
  public static final double SHOOTER_WHEEL_RADIUS = 0.051;
  public static final double LIMELIGHT_HEIGHT = 0.61;
  public static final double LIMELIGHT_Y_ANGLE = 24;

  public static final int CLIMB_ARM_MOTOR_ID = 14; // TODO
  public static final int CLIMB_ELEVATOR_MOTOR_ID = 15; // TODO
  public static final double CLIMB_ARM_kP = 0.0;
  public static final double CLIMB_ARM_kI = 0.0;
  public static final double CLIMB_ARM_kD = 0.0;
  public static final double CLIMB_ARM_kIZ = 0.0;
  public static final double CLIMB_ARM_kFF = 0.0;
  public static final double CLIMB_ARM_OUTPUT = 0.0;
  public static final double CLIMB_ARM_MAX_VELOCITY = 0.0;
  public static final double CLIMB_ARM_MIN_VELOCITY = 0.0;
  public static final double CLIMB_ARM_MAX_ACCELERATION = 0.0;
  public static final double CLIMB_ARM_ALLOWED_ERROR = 0.0;
  public static final double CLIMB_ELEVATOR_kP = 0.0;
  public static final double CLIMB_ELEVATOR_kI = 0.0;
  public static final double CLIMB_ELEVATOR_kD = 0.0;
  public static final double CLIMB_ELEVATOR_kIZ = 0.0;
  public static final double CLIMB_ELEVATOR_kFF = 0.0;
  public static final double CLIMB_ELEVATOR_OUTPUT = 0.0;
  public static final double CLIMB_ELEVATOR_MAX_VELOCITY = 0.0;
  public static final double CLIMB_ELEVATOR_MIN_VELOCITY = 0.0;
  public static final double CLIMB_ELEVATOR_MAX_ACCELERATION = 0.0;
  public static final double CLIMB_ELEVATOR_ALLOWED_ERROR = 0.0;

  public static final int CONTROL_PANEL_MOTOR_ID = 12;
  public static final int CONTROL_PANEL_ROTATIONS = 30;
  public static final double CONTROL_PANEL_SPEED = 0.3;

  public static final int ROLLING_AVERAGE_SIZE = 5;
}
