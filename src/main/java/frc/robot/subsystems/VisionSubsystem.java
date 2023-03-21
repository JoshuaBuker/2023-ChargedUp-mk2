package frc.robot.subsystems;

import Common.Limelight;
import Common.LimelightPipeline;
import Common.Target;
import Common.Target.DetectionType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionSubsystem extends SubsystemBase {


//====================== Pipelines ==================================
public static class Pipelines {
  public static LimelightPipeline defaultPipeline = new LimelightPipeline(4, false);
  public static LimelightPipeline reflectiveUpperPipeline = new LimelightPipeline(0, true);
  public static LimelightPipeline reflectiveLowerPipeline = new LimelightPipeline(5, true);
  public static LimelightPipeline apriltagPipeline = new LimelightPipeline(1, false);
  public static LimelightPipeline conePipeline = new LimelightPipeline(2, false);
  public static LimelightPipeline cubePipeline = new LimelightPipeline(3, false);
}

//====================== Limelights =================================
public static class Limelights {
  public static Limelight defaultLimelight = new Limelight("limelight", Pipelines.defaultPipeline, 15, 15);
}

//====================== Targets ===================================
public static class Targets {
  public static Target gridTag = new Target("gridTag", 18, DetectionType.AprilTag, 0.5);
  public static Target substationTag = new Target("substationTag",28, DetectionType.AprilTag, 0.5);
  public static Target gridTapeUpper = new Target("gridTapeUpper",24, DetectionType.Reflective, 0.5);
  public static Target gridTapeLower = new Target("gridTapeLower",43, DetectionType.Reflective, 0.5);
  public static Target cone = new Target("cone", 4, DetectionType.GamePiece, 0.5);
  public static Target cube = new Target("cube", 4, DetectionType.GamePiece, 0.5);
}




public static void setPipeline(Limelight limelight, LimelightPipeline pipeline) {
  limelight.setPipeline(pipeline);
}
  public VisionSubsystem() {}
  @Override

  public void periodic() {}

 
}
