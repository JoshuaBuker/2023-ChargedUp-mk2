package frc.robot;

/** This is a collection of all setpoints for each subsystem */
public class SetpointConstructors {

    public static class ShoulderSetpoint {

        private final double angle;
    
        public ShoulderSetpoint(double angle) {
          this.angle = angle;
        }
    
        public double getSetpointAngle() {
          return angle;
        }
      }




}
