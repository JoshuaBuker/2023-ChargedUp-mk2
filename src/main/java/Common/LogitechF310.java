package Common;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;

/** 
 * <p>Creates a Logitech F310 specific {@link Joystick} including methods for every button and axis</p>
 */
public class LogitechF310  {

    private Joystick joy;

//==============================================================================
//=============================== Joystick IDs ===================================
    
    private final int btnA = 1;
    private final int btnB = 2;
    private final int btnX = 3;
    private final int btnY = 4;
    private final int btnL1 = 5;
    private final int btnR1 = 6;   
    private final int btnSelect = 8;       
    private final int btnR3 = 9;
    private final int btnL3 = 10;
    private final int leftXAxis = 0;
    private final int leftYAxis = 1;
    private final int leftTrigger = 2;
    private final int rightTrigger = 3;
    private final int rightXAxis = 4;
    private final int rightYAxis = 5;

//==============================================================================
//============================== Constructor ===================================

    public LogitechF310(int port) {
        this.joy = new Joystick(port);
    }

//==============================================================================
//============================ Create Buttons ==================================

    JoystickButton buttonA = new JoystickButton(joy, btnA);
    JoystickButton buttonB = new JoystickButton(joy, btnB);
    JoystickButton buttonX = new JoystickButton(joy, btnX);
    JoystickButton buttonY = new JoystickButton(joy, btnY);
    JoystickButton buttonL1 = new JoystickButton(joy, btnL1);
    JoystickButton buttonR1 = new JoystickButton(joy, btnR1);
    JoystickButton buttonSelect = new JoystickButton(joy, btnSelect);
    JoystickButton buttonR3 = new JoystickButton(joy, btnR3);
    JoystickButton buttonL3 = new JoystickButton(joy, btnL3);

    POVButton dPadUp = new POVButton(joy, 0);
    POVButton dPadRight = new POVButton(joy, 90);
    POVButton dPadDown = new POVButton(joy, 180);
    POVButton dPadLeft = new POVButton(joy, 270);

//==============================================================================
//================================ Getters =====================================
    
    public Joystick getJoy() {
        return joy;
    }
    public JoystickButton getButtonA() {
        return buttonA;
    }
    public JoystickButton getButtonB() {
        return buttonB;
    }
    public JoystickButton getButtonX() {
        return buttonX;
    }
    public JoystickButton getButtonY() {
        return buttonY;
    }
    public JoystickButton getButtonL1() {
        return buttonL1;
    }
    public JoystickButton getButtonR1() {
        return buttonR1;
    }
    public JoystickButton getButtonSelect() {
        return buttonSelect;
    }
    public JoystickButton getButtonR3() {
        return buttonR3;
    }
    public JoystickButton getButtonL3() {
        return buttonL3;
    }
    public POVButton getDPadUp() {
        return dPadUp;
    }
    public POVButton getDPadRight() {
        return dPadRight;
    }
    public POVButton getDPadDown() {
        return dPadDown;
    }
    public POVButton getDPadLeft() {
        return dPadLeft;
    }
    public double getLeftXAxis() {
        return joy.getRawAxis(leftXAxis);
    }
    public double getLeftYAxis() {
        return joy.getRawAxis(leftYAxis);
    }
    public double getRightXAxis() {
        return joy.getRawAxis(rightXAxis);
    }
    public double getRightYAxis() {
        return joy.getRawAxis(rightYAxis);
    }
    public double getLeftTrigger() {
        return joy.getRawAxis(leftTrigger);
    }
    public double getRightTrigger() {
        return joy.getRawAxis(rightTrigger);
    }
    

}
