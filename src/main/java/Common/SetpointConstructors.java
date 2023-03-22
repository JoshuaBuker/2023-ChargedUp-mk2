package Common;

import java.util.ArrayList;

import Common.ConstraintConstructors.PitchConstraint;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

/** This is a collection of all setpoints for each subsystem */
public class SetpointConstructors {

    public static class PitchSetpoint {

        private final double angle;
        private final double tolerance;
    
        public PitchSetpoint(double angleDegrees, double toleranceDegrees) {
          this.angle = angleDegrees;
          this.tolerance = toleranceDegrees;
        }
    
        public double getSetpointAngleDegrees() {
          return angle;
        }
        
        public double getToleranceDegrees() {
          return tolerance;
        }

      
    }

    public static class DoubleSolenoidGroupSetpoint {

      private static ArrayList<DoubleSolenoidGroupSetpoint> solenoidSetpoints;

      Value cylinderValueOne;
      Value cylinderValueTwo;
      PitchConstraint constraint;

      public DoubleSolenoidGroupSetpoint(Value forCylinderOne, Value forCylinderTwo) {
        this.cylinderValueOne = forCylinderOne;
        this.cylinderValueTwo = forCylinderTwo;

        solenoidSetpoints.add(this);
      }

      public DoubleSolenoidGroupSetpoint(Value forCylinderOne, Value forCylinderTwo, PitchConstraint constraint) {
        this.cylinderValueOne = forCylinderOne;
        this.cylinderValueTwo = forCylinderTwo;
        this.constraint = constraint;
      }
      
      public Value getCylinderValueOne() {
        return cylinderValueOne;
      }

      public Value getCylinderValueTwo() {
        return cylinderValueTwo;
      }

      public static ArrayList<DoubleSolenoidGroupSetpoint> getSetpoints() {
        return solenoidSetpoints;
      }
    }



}
