package Common;

/** Create custom contraints here */
public class ConstraintConstructors {


    public static class PitchConstraint {

        private final double UPPER_CONSTRAINT;
        private final double LOWER_CONSTRAINT;
    
        public PitchConstraint(double LOWER_CONSTRAINT, double UPPER_CONSTRAINT) {
          this.LOWER_CONSTRAINT = LOWER_CONSTRAINT;
          this.UPPER_CONSTRAINT = UPPER_CONSTRAINT;
        }
    
        public double getLowerConstraint() {
            return LOWER_CONSTRAINT;
          }

        public double getUpperConstraint() {
          return UPPER_CONSTRAINT;
        }
      }
}
