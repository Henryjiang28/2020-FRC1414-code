package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.revrobotics.ColorSensorV3;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorMatch;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;

public class ControlPanel extends SubsystemBase {
  private final I2C.Port i2cPort = I2C.Port.kOnboard;

  private final ColorSensorV3 colorSensor = new ColorSensorV3(i2cPort);
  private final ColorMatch colorMatcher = new ColorMatch();

  private final TalonSRX controlPanelMotor = new TalonSRX(Constants.CONTROL_PANEL_MOTOR_ID);

  double prevColor;
  double targetColor;
  int colorCount = 0;

  public ControlPanel() {
    this.controlPanelMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 0);
  }

  public void runMotor(double throttle) {
    this.controlPanelMotor.set(ControlMode.PercentOutput, throttle);
  }

  public void stopMotor() {
    this.controlPanelMotor.set(ControlMode.PercentOutput, 0.0);
  }

  public int colorMatch() {
    int colorValue;

    final Color blueTarget = ColorMatch.makeColor(0.143, 0.427, 0.429);
    final Color greenTarget = ColorMatch.makeColor(0.197, 0.561, 0.240);
    final Color redTarget = ColorMatch.makeColor(0.561, 0.232, 0.114);
    final Color yellowTarget = ColorMatch.makeColor(0.361, 0.524, 0.113);

    Color detectedColor = colorSensor.getColor();
    ColorMatchResult match = colorMatcher.matchClosestColor(detectedColor);

    if (match.color == blueTarget) {
      colorValue = 1;
    } else if (match.color == redTarget) {
      colorValue = 2;
    } else if (match.color == greenTarget) {
      colorValue = 3;
    } else if (match.color == yellowTarget) {
      colorValue = 4;
    } else {
      colorValue = 0;
    }

    return colorValue;
  }

  public void positionControl() {
    String gameData = DriverStation.getInstance().getGameSpecificMessage();
    
    int targetColorValue = 0;

    // B - R, G - Y
    // 1 - B, 2 - R, 3 - G, 4 - Y
    if(gameData.length() > 0) {
      switch (gameData.charAt(0)) {
        case 'B' :
          //Blue case code
          targetColorValue = 2;
          break;
        case 'G' :
          //Green case code
          targetColorValue = 4;
          break;
        case 'R' :
          //Red case code
          targetColorValue = 1;
          break;
        case 'Y' :
          //Yellow case code
          targetColorValue = 3;
          break;
        default :
          targetColorValue = 0;
          break;
      }
    } else {
      targetColorValue = 0;
    }
    
    if (colorMatch() != targetColorValue && targetColorValue > 0){
      runMotor(Constants.CONTROL_PANEL_SPEED);
    }
    else {
      runMotor(0.0);
    }
  }

  public void rotationControl() {
    
    double currentColor = colorMatch();
    
    if (colorCount < Constants.CONTROL_PANEL_ROTATIONS) {
      runMotor(Constants.CONTROL_PANEL_SPEED);
      if(currentColor != prevColor) {
        colorCount++;
      }
    } else {
      runMotor(0.0);
    }
    prevColor = currentColor;
  }
  
  @Override
  public void periodic() {
    SmartDashboard.putNumber("Control Panel Speed", this.controlPanelMotor.getMotorOutputPercent());
    SmartDashboard.putNumber("Sensor Color", this.colorMatch());
    SmartDashboard.putString("Target Color", DriverStation.getInstance().getGameSpecificMessage());
  }
}