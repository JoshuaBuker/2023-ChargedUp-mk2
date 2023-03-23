// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Defaults;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmExtentionSubsystem;

public class DefaultExtentionCommand extends CommandBase {

  ArmExtentionSubsystem arm;
  DoubleSupplier xAxisSupplier;
  double xAxis;


  public DefaultExtentionCommand(ArmExtentionSubsystem arm, DoubleSupplier xAxisSupplier) {
    this.arm = arm;
    this.xAxisSupplier = xAxisSupplier;

    addRequirements(arm);
  }

  @Override
  public void initialize() {
    xAxis = xAxisSupplier.getAsDouble();

    if (xAxis > 0.5) {
      arm.next();
    } else if (xAxis < -0.5) {
      arm.previous();
    }

  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {}
}
