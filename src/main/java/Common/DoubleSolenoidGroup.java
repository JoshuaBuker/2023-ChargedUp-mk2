package Common;

import Common.ConstraintConstructors.PitchConstraint;
import Common.SetpointConstructors.DoubleSolenoidGroupSetpoint;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

/** A custom group class for Double Solenoids */
public class DoubleSolenoidGroup {
    private DoubleSolenoid solenoidOne;
    private DoubleSolenoid solenoidTwo;

    public DoubleSolenoidGroup(DoubleSolenoid solenoidOne, DoubleSolenoid solenoidTwo) {
        this.solenoidOne = solenoidOne;
        this.solenoidTwo = solenoidTwo;
    }

    public DoubleSolenoid getSolenoidOne() {
        return solenoidOne;
    }

    public DoubleSolenoid getSolenoidTwo() {
        return solenoidTwo;
    }


    public void set(DoubleSolenoidGroupSetpoint setpoint) {
        solenoidOne.set(setpoint.getCylinderValueOne());
        solenoidTwo.set(setpoint.getCylinderValueTwo());
    }

    public void setWithConstraint(DoubleSolenoidGroupSetpoint setpoint, double currentValue, PitchConstraint constraint) {
        if (currentValue > constraint.getLowerConstraint() && currentValue < constraint.getUpperConstraint()) {
            solenoidOne.set(setpoint.getCylinderValueOne());
            solenoidTwo.set(setpoint.getCylinderValueTwo());
        }
        
    }

    public void disable() {
        solenoidOne.set(Value.kOff);
        solenoidTwo.set(Value.kOff);
    }
}
