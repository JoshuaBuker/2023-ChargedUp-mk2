package frc.robot.subsystems;

import Common.DoubleSolenoidGroup;
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

  
  public ArmExtentionSubsystem() {
    armExtentionGroup.disable();
  }

    public int getIndexCounter() {
    return indexCounter;
  }

  public void IndexCounter(int indexCounter) {
    this.indexCounter = indexCounter;
  }


  @Override
  public void periodic() {}
}
