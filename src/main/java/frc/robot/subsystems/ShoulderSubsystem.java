package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShoulderConstants;
import frc.robot.Setpoints.ShoulderConstraints;

public class ShoulderSubsystem extends SubsystemBase {

  private CANSparkMax leftMotor = new CANSparkMax(ShoulderConstants.SHOULDER_MOTOR_LEFT, MotorType.kBrushless);
  private CANSparkMax rightMotor = new CANSparkMax(ShoulderConstants.SHOULDER_MOTOR_RIGHT, MotorType.kBrushless);
  private MotorControllerGroup shoulderMotors = new MotorControllerGroup(leftMotor, rightMotor);
  private DutyCycleEncoder encoder = new DutyCycleEncoder(ShoulderConstants.SHOULDER_ENCODER);
  private double currentAngle;


  public ShoulderSubsystem() {
    leftMotor.setInverted(false);
    rightMotor.setInverted(true);
  }

  public double getEncoderAngle() {
    return encoder.getDistance() + ShoulderConstants.ENCODER_OFFSET;
  }

  public double createManualSetpoint(double yAxis, double scalingFactor) {
    if ((currentAngle + (yAxis * scalingFactor) < (ShoulderConstraints.manualControl.getLowerConstraint()) + scalingFactor) || (currentAngle + (yAxis * scalingFactor) > (ShoulderConstraints.manualControl.getUpperConstraint()) - scalingFactor)) {
      return currentAngle;
    } else {
      return (currentAngle + (yAxis * scalingFactor));
    }
  }

  public void runShoulderMotors(double val) {
    shoulderMotors.set(val);
  }
  

  @Override
  public void periodic() {}

}
