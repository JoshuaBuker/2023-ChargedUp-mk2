// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Vision;

import java.util.function.DoubleSupplier;

import Common.Limelight;
import Common.Target;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.VisionContainer;
import frc.robot.subsystems.VisionContainer.Pipelines;

public class alignToTarget extends CommandBase {

  DrivetrainSubsystem drivetrain;
  VisionContainer vision;
  Limelight limelight;
  Target target;

  PIDController pidForward;
  PIDController pidStrafe;
  PIDController pidRotate;

  DoubleSupplier strafeAxis;

  public alignToTarget(DrivetrainSubsystem drivetrain, VisionContainer vision, Limelight limelight, Target target, DoubleSupplier strafeAxis) {
    this.drivetrain = drivetrain;
    this.limelight = limelight;
    this.vision = vision;
    this.target = target;
    this.strafeAxis = strafeAxis;

    addRequirements(drivetrain);
  }

  @Override
  public void initialize() {
    pidForward.setPID(0.05, 0, 0);
    pidForward.setTolerance(target.getAlignmentTolerance());

    pidStrafe.setPID(0.05, 0, 0);
    pidStrafe.setTolerance(target.getAlignmentTolerance());

    pidRotate.setPID(0.05, 0, 0);
    pidRotate.setTolerance(target.getAlignmentTolerance());

    
      switch (target.getName()) {
        // Reflective
        case "gridTapeLower": VisionContainer.setPipeline(limelight, Pipelines.reflectiveLowerPipeline); break;
        case "gridTapeUpper": VisionContainer.setPipeline(limelight, Pipelines.reflectiveUpperPipeline); break;
        // AprilTags
        case "gridTag": VisionContainer.setPipeline(limelight, Pipelines.apriltagPipeline); break;
        case "substationTag": VisionContainer.setPipeline(limelight, Pipelines.apriltagPipeline); break;
        // Game Pieces
        case "cone": VisionContainer.setPipeline(limelight, Pipelines.conePipeline);
        case "cube": VisionContainer.setPipeline(limelight, Pipelines.cubePipeline);
        // Default
        default: VisionContainer.setPipeline(limelight, Pipelines.defaultPipeline); break;

      }
  }

  @Override
  public void execute() {

    drivetrain.drive(new ChassisSpeeds(
      pidForward.calculate(limelight.distanceFromTargetInMeters(limelight, target), 1),
      strafeAxis.getAsDouble(),
      pidRotate.calculate(limelight.getHorizontalError(), 0)
    ));

  }

  @Override
  public void end(boolean interrupted) {
    drivetrain.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
  }

  @Override
  public boolean isFinished() {
    return (pidForward.atSetpoint() && pidRotate.atSetpoint());
  }
}
