package frc.robot.subsystems;

import java.util.ArrayList;

import Common.DoubleSolenoidGroup;
import Common.SetpointConstructors.DoubleSolenoidGroupSetpoint;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmExtentionConstants;
import frc.robot.Constants.SuperstructureConstants;

public class ArmExtentionSubsystem extends SubsystemBase {

  private int indexCounter = 0;

  private DoubleSolenoid lowerCylinders = new DoubleSolenoid(SuperstructureConstants.PNEUMATICS_HUB_ID, PneumaticsModuleType.REVPH, ArmExtentionConstants.LOWER_CYLINDER[0], ArmExtentionConstants.LOWER_CYLINDER[1]);
  private DoubleSolenoid upperCylinders = new DoubleSolenoid(SuperstructureConstants.PNEUMATICS_HUB_ID, PneumaticsModuleType.REVPH, ArmExtentionConstants.UPPER_CYLINDER[0], ArmExtentionConstants.UPPER_CYLINDER[1]);
  
  private DoubleSolenoidGroup armExtentionGroup = new DoubleSolenoidGroup(lowerCylinders, upperCylinders);

  private ArrayList<DoubleSolenoidGroupSetpoint> setpoints = DoubleSolenoidGroupSetpoint.getSetpoints();

  
  public ArmExtentionSubsystem() {
    armExtentionGroup.disable();
  }

    public int getIndexCounter() {
    return indexCounter;
  }

  public void setArmToSetpoint(DoubleSolenoidGroupSetpoint setpoint) {
    armExtentionGroup.set(setpoint);
  }

  public void next() {
    if (indexCounter < setpoints.size()) {
      indexCounter++;
      armExtentionGroup.set(setpoints.get(indexCounter));
    }
  }

  public void previous() {
    if (indexCounter > 0) {
      indexCounter--;
      armExtentionGroup.set(setpoints.get(indexCounter));
    } else {
      armExtentionGroup.set(setpoints.get(0));
    }
  }


  @Override
  public void periodic() {}
}
