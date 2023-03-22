package frc.robot.commands.Defaults;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShoulderSubsystem;

public class DefaultShoulderCommand extends CommandBase {

  private ShoulderSubsystem shoulder;
  private double scalingFactor;
  private DoubleSupplier yAxisSupplier;
  private double currentAngle;
  private double yAxis;

  private PIDController pid = new PIDController(0.02, 0.0, 0.0);

  
  public DefaultShoulderCommand(ShoulderSubsystem shoulder, double scalingFactor, DoubleSupplier yAxisSupplier) {
    this.scalingFactor = scalingFactor;
    this.yAxisSupplier = yAxisSupplier;


    addRequirements(shoulder);
  }

  @Override
  public void initialize() {
    pid.setTolerance(1);
  }

  @Override
  public void execute() {
    currentAngle = shoulder.getEncoderAngle();
    yAxis = yAxisSupplier.getAsDouble();

    if (yAxis > 0.1 || yAxis < -0.1) {
        shoulder.runShoulderMotors(pid.calculate(currentAngle, shoulder.createManualSetpoint(yAxis, scalingFactor)));
      }
    }

  @Override
  public void end(boolean interrupted) {
    shoulder.runShoulderMotors(0.0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
