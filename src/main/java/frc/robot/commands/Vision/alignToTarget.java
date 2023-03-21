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
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.VisionSubsystem.Pipelines;

public class alignToTarget extends CommandBase {

  DrivetrainSubsystem drivetrain;
  VisionSubsystem vision;
  Limelight limelight;
  Target target;

  PIDController pidForward;
  PIDController pidStrafe;
  PIDController pidRotate;

  DoubleSupplier strafeAxis;

  public alignToTarget(DrivetrainSubsystem drivetrain, VisionSubsystem vision, Limelight limelight, Target target, DoubleSupplier strafeAxis) {
    this.drivetrain = drivetrain;
    this.limelight = limelight;
    this.vision = vision;
    this.target = target;
    this.strafeAxis = strafeAxis;

    addRequirements(drivetrain, vision);
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
        case "gridTapeLower": VisionSubsystem.setPipeline(limelight, Pipelines.reflectiveLowerPipeline); break;
        case "gridTapeUpper": VisionSubsystem.setPipeline(limelight, Pipelines.reflectiveUpperPipeline); break;
        // AprilTags
        case "gridTag": VisionSubsystem.setPipeline(limelight, Pipelines.apriltagPipeline); break;
        case "substationTag": VisionSubsystem.setPipeline(limelight, Pipelines.apriltagPipeline); break;
        // Game Pieces
        case "cone": VisionSubsystem.setPipeline(limelight, Pipelines.conePipeline);
        case "cube": VisionSubsystem.setPipeline(limelight, Pipelines.cubePipeline);
        // Default
        default: VisionSubsystem.setPipeline(limelight, Pipelines.defaultPipeline); break;

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
