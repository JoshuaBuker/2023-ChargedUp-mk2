package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import Common.ArcadeControls;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.ShoulderConstants;
import frc.robot.SetpointConstructors.ShoulderSetpoint;

public class ShoulderSubsystem extends SubsystemBase {

  @SuppressWarnings("unused")
  public static class ShoulderSetpoints {
    private static ShoulderSetpoint home = new ShoulderSetpoint(5);
    private static ShoulderSetpoint forward = new ShoulderSetpoint(50);
  }

  private CANSparkMax leftMotor = new CANSparkMax(ShoulderConstants.SHOULDER_MOTOR_LEFT, MotorType.kBrushless);
  private CANSparkMax rightMotor = new CANSparkMax(ShoulderConstants.SHOULDER_MOTOR_RIGHT, MotorType.kBrushless);
  private MotorControllerGroup shoulderMotors = new MotorControllerGroup(leftMotor, rightMotor);
  private DutyCycleEncoder encoder = new DutyCycleEncoder(ShoulderConstants.SHOULDER_ENCODER);
  private PIDController pid = new PIDController(0.02, 0.0, 0.0);
  private ArcadeControls arcadeControls = RobotContainer.getArcadeControls();

  private boolean autoEnabled;
  private ShoulderSetpoint autoSetpoint;
  private double currentAngle;

  

  public ShoulderSubsystem() {
    setAutoSetpoint(ShoulderSetpoints.home);

    pid.setTolerance(ShoulderConstants.ANGLE_TOLERANCE);
    leftMotor.setInverted(false);
    rightMotor.setInverted(true);
    autoEnabled = false;
  }

  public void setAutoSetpoint(ShoulderSetpoint autoSetpoint) {
    this.autoSetpoint = autoSetpoint;
  }

  public void enableAuto(boolean enabled) {
      this.autoEnabled = enabled;
  }

  public double getEncoderAngle() {
    return encoder.getDistance() + ShoulderConstants.ENCODER_OFFSET;
  }

  public double createManualSetpoint(double yAxis, double scalingFactor) {
    if ((currentAngle + (yAxis * scalingFactor) < ShoulderConstants.LOWER_CONSTRAINT) || (currentAngle + (yAxis * scalingFactor) > ShoulderConstants.UPPER_CONSTRAINT)) {
      return currentAngle;
    }
    return (currentAngle + (yAxis * scalingFactor));
  }
  

  @Override
  public void periodic() {
    currentAngle = getEncoderAngle();

    if (arcadeControls.isYAxisActive()) {
      shoulderMotors.set(pid.calculate(currentAngle, createManualSetpoint(arcadeControls.getY(), 25)));
    } else if (autoEnabled) {
      shoulderMotors.set(pid.calculate(currentAngle, autoSetpoint.getSetpointAngle()));
    }

  }
}
