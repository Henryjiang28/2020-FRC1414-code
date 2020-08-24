package frc.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climb extends SubsystemBase {

  private final CANSparkMax elevatorMotor = new CANSparkMax(Constants.CLIMB_ELEVATOR_MOTOR_ID, MotorType.kBrushless);
  private final CANSparkMax armMotor = new CANSparkMax(Constants.CLIMB_ARM_MOTOR_ID, MotorType.kBrushless);
  private final CANEncoder armEncoder, elevatorEncoder;
  private final CANPIDController armController, elevatorController;

  public static enum ArmState {
    Raising, Lowering, Stationary, BottomedOut, ToppedOut,
  }

  public ArmState getArmState() {
      return armState;
  }

  private void setArmState(ArmState newState) {
      this.armState= newState;
  }

  private volatile ArmState armState = ArmState.Stationary;

  public enum ArmPosition {
      Starting(0), Climb(0), ControlPanel(0);
      
      private int position;

      ArmPosition(int encPos) {
          this.position = encPos; // the position is set to the encoder value (which is associated with a position)
      }

      public int getPosition() {
          return this.position; //retrieving the position
      }
  }

  private volatile ArmPosition armPosition = ArmPosition.Starting;

  public ArmPosition getArmPosition() {
      return armPosition;
  }

  public void setArmPosition(ArmPosition newPos) {
      this.armPosition = newPos;
  }

  public static enum ElevatorState {
    Raising, Lowering, Stationary, BottomedOut, ToppedOut,
  }

  public ElevatorState getElevatorState() {
      return elevatorState;
  }

  private void setElevatorState(ElevatorState newState) {
      this.elevatorState = newState;
  }

  private volatile ElevatorState elevatorState = ElevatorState.Stationary;

  public enum ElevatorPosition {
      Starting(0), Climb(0), ControlPanel(0);
      
      private int position;

      ElevatorPosition(int encPos) {
          this.position = encPos; // the position is set to the encoder value (which is associated with a position)
      }

      public int getPosition() {
          return this.position; //retrieving the position
      }
  }

  private volatile ElevatorPosition elevatorPosition = ElevatorPosition.Starting;

  public ElevatorPosition getElevatorPosition() {
      return elevatorPosition;
  }

  public void setElevatorPosition(ElevatorPosition newPos) {
      this.elevatorPosition = newPos;
  }

  public Climb() {
    this.elevatorEncoder = this.elevatorMotor.getEncoder();
    this.elevatorController = this.elevatorMotor.getPIDController();

    this.armEncoder = this.armMotor.getEncoder();
    this.armController = this.armMotor.getPIDController();
  
    this.elevatorMotor.setIdleMode(IdleMode.kBrake);
    this.armMotor.setIdleMode(IdleMode.kBrake);

    this.armController.setP(Constants.CLIMB_ARM_kP);
    this.armController.setI(Constants.CLIMB_ARM_kI);
    this.armController.setD(Constants.CLIMB_ARM_kD);
    this.armController.setIZone(Constants.CLIMB_ARM_kIZ);
    this.armController.setFF(Constants.CLIMB_ARM_kFF);
    this.armController.setOutputRange(-Constants.CLIMB_ARM_OUTPUT, Constants.CLIMB_ARM_OUTPUT);

    this.armController.setSmartMotionMaxVelocity(Constants.CLIMB_ARM_MAX_VELOCITY, 0);
    this.armController.setSmartMotionMinOutputVelocity(Constants.CLIMB_ARM_MIN_VELOCITY, 0);
    this.armController.setSmartMotionMaxAccel(Constants.CLIMB_ARM_MAX_ACCELERATION, 0);
    this.armController.setSmartMotionAllowedClosedLoopError(Constants.CLIMB_ARM_ALLOWED_ERROR, 0);

    this.elevatorController.setP(Constants.CLIMB_ELEVATOR_kP);
    this.elevatorController.setI(Constants.CLIMB_ELEVATOR_kI);
    this.elevatorController.setD(Constants.CLIMB_ELEVATOR_kD);
    this.elevatorController.setIZone(Constants.CLIMB_ELEVATOR_kIZ);
    this.elevatorController.setFF(Constants.CLIMB_ELEVATOR_kFF);
    this.elevatorController.setOutputRange(-Constants.CLIMB_ELEVATOR_OUTPUT, Constants.CLIMB_ELEVATOR_OUTPUT);

    this.elevatorController.setSmartMotionMaxVelocity(Constants.CLIMB_ELEVATOR_MAX_VELOCITY, 0);
    this.elevatorController.setSmartMotionMinOutputVelocity(Constants.CLIMB_ELEVATOR_MIN_VELOCITY, 0);
    this.elevatorController.setSmartMotionMaxAccel(Constants.CLIMB_ELEVATOR_MAX_ACCELERATION, 0);
    this.elevatorController.setSmartMotionAllowedClosedLoopError(Constants.CLIMB_ELEVATOR_ALLOWED_ERROR, 0);
  }

  public void startArmMotionMagic(ArmPosition pos) {
    if (this.getArmEncoder() > pos.getPosition()) {
        setArmState(ArmState.Lowering);
    } else if (this.getArmEncoder() < pos.getPosition()) {
        setArmState(ArmState.Raising);
    }

    this.armController.setReference(pos.getPosition(), ControlType.kSmartMotion);
  }

  public void checkArmMotionMagicTermination(ArmPosition pos) {
      if (pos == ArmPosition.Starting) {
          armState = ArmState.Stationary;
          // stopArm();
          armPosition = pos;
      } else if (Math.abs(pos.getPosition() - this.getArmEncoder()) <= Constants.CLIMB_ARM_ALLOWED_ERROR) {
          armState = ArmState.Stationary;
          // stopArm();
          armPosition = pos;
      }
  }

  public void stopArm() {
      this.armMotor.set(0.0);
  }

  public void directArm(double pow) {
      if (getArmState() == ArmState.BottomedOut && pow < 0.0) {
          return;
      }
      if (getArmState() == ArmState.ToppedOut && pow > 0.0) {
          return;
      }
      if (pow > 0.0) {
          setArmState(ArmState.Raising);
      }
      if (pow < 0.0) {
          setArmState(ArmState.Lowering);
      }
      if (pow == 0.0) {
          setArmState(ArmState.Stationary);
      }
      this.armMotor.set(pow);
  }

  public double getArmEncoder() {
    return this.armEncoder.getPosition();
  }

  public void startElevatorMotionMagic(ElevatorPosition pos) {
    if (this.getElevatorEncoder() > pos.getPosition()) {
        setElevatorState(ElevatorState.Lowering);
    } else if (this.getElevatorEncoder() < pos.getPosition()) {
        setElevatorState(ElevatorState.Raising);
    }

    this.elevatorController.setReference(pos.getPosition(), ControlType.kSmartMotion);
  }

  public void checkElevatorMotionMagicTermination(ElevatorPosition pos) {
      if (pos == ElevatorPosition.Starting) {
          elevatorState = ElevatorState.Stationary;
          // stopElevator();
          elevatorPosition = pos;
      } else if (Math.abs(pos.getPosition() - this.getElevatorEncoder()) <= Constants.CLIMB_ELEVATOR_ALLOWED_ERROR) {
          elevatorState = ElevatorState.Stationary;
          // stopElevator();
          elevatorPosition = pos;
      }
  }

  public void stopElevator() {
      this.elevatorMotor.set(0.0);
  }

  public void directElevator(double pow) {
      if (getElevatorState() == ElevatorState.BottomedOut && pow < 0.0) {
          return;
      }
      if (getElevatorState() == ElevatorState.ToppedOut && pow > 0.0) {
          return;
      }
      if (pow > 0.0) {
          setElevatorState(ElevatorState.Raising);
      }
      if (pow < 0.0) {
          setElevatorState(ElevatorState.Lowering);
      }
      if (pow == 0.0) {
          setElevatorState(ElevatorState.Stationary);
      }
      this.elevatorMotor.set(pow);
  }

  public double getElevatorEncoder() {
    return this.elevatorEncoder.getPosition();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Arm Encoder", this.getArmEncoder());
    SmartDashboard.putNumber("Elevator Encoder", this.getElevatorEncoder());
    SmartDashboard.putString("Arm State", this.getArmPosition().toString());
    SmartDashboard.putString("Elevator State", this.getElevatorPosition().toString());
  }
}