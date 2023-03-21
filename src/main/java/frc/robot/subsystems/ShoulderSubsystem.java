package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShoulderConstants;
import frc.robot.Setpoints.ShoulderSetpoint;

public class ShoulderSubsystem extends SubsystemBase {


  public static class ShoulderSetpoints {
    private static ShoulderSetpoint home = new ShoulderSetpoint(5);
    private static ShoulderSetpoint forward = new ShoulderSetpoint(50);
  }

  private CANSparkMax leftMotor = new CANSparkMax(ShoulderConstants.SHOULDER_MOTOR_LEFT, MotorType.kBrushless);
  private CANSparkMax rightMotor = new CANSparkMax(ShoulderConstants.SHOULDER_MOTOR_RIGHT, MotorType.kBrushless);
  private MotorControllerGroup shoulderMotors = new MotorControllerGroup(leftMotor, rightMotor);
  private DutyCycleEncoder encoder = new DutyCycleEncoder(ShoulderConstants.SHOULDER_ENCODER);
  private PIDController pid = new PIDController(0.02, 0.0, 0.0);

  private boolean enabled;
  private ShoulderSetpoint setpoint;

  

  public ShoulderSubsystem() {
    pid.setTolerance(ShoulderConstants.ANGLE_TOLERANCE);
    leftMotor.setInverted(false);
    rightMotor.setInverted(true);
    enabled = false;
  }



  public void setSetpoint(ShoulderSetpoint setpoint) {
    this.setpoint = setpoint;
  }

  public void armEnabled(boolean enabled) {
      this.enabled = enabled;
  }

  public double getEncoderAngle() {
    return encoder.getDistance() + ShoulderConstants.ENCODER_OFFSET;
  }
  

  @Override
  public void periodic() {

    if (enabled) {
      shoulderMotors.set(pid.calculate(getEncoderAngle(), setpoint.getSetpointAngle()));
    } else {
      shoulderMotors.set(0.0);
    }


  }
}
