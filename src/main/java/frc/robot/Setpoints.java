package frc.robot;

import java.util.ArrayList;

import Common.ConstraintConstructors.PitchConstraint;
import Common.SetpointConstructors.DoubleSolenoidGroupSetpoint;
import Common.SetpointConstructors.PitchSetpoint;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

public class Setpoints {

    @SuppressWarnings("unused")
  public static class ArmExtentionSetpoints {
    public static DoubleSolenoidGroupSetpoint retracted = new DoubleSolenoidGroupSetpoint(Value.kReverse, Value.kReverse, ShoulderConstraints.retractable);
    public static DoubleSolenoidGroupSetpoint lowerExtended = new DoubleSolenoidGroupSetpoint(Value.kForward, Value.kReverse);
    public static DoubleSolenoidGroupSetpoint upperExtended = new DoubleSolenoidGroupSetpoint(Value.kReverse, Value.kForward);
    public static DoubleSolenoidGroupSetpoint fullyExtended = new DoubleSolenoidGroupSetpoint(Value.kForward, Value.kForward);

    // Always keep this at the bottom
    public static ArrayList<DoubleSolenoidGroupSetpoint> solenoidSetpoints = DoubleSolenoidGroupSetpoint.getSetpoints();
  }

  @SuppressWarnings("unused")
  public static class ShoulderSetpoints {
    public static PitchSetpoint home = new PitchSetpoint(5, 1);
    public static PitchSetpoint forward = new PitchSetpoint(50, 1);
  }

  @SuppressWarnings("unused")
  public static class ShoulderConstraints {
    public static PitchConstraint manualControl = new PitchConstraint(5, 290);
    public static PitchConstraint retractable = new PitchConstraint(15, 280);
  }

}
