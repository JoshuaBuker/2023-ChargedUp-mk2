package Common;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/** Creates a custom Joystick object for our Arcade driverstation */
public class ArcadeControls {

    Joystick joy;
//==============================================================================
//=============================== Joystick IDs ===================================
    private final int SMALL_TOP = 3;
    private final int SMALL_BOTTOM = 4;

    private final int RIGHT_TOP = 5;
    private final int RIGHT_MIDDLE_UPPER = 6;
    private final int RIGHT_MIDDLE_LOWER = 7;
    private final int RIGHT_BOTTOM = 8;

    private final int LEFT_TOP = 9;
    private final int LEFT_MIDDLE_UPPER = 10;
    private final int LEFT_MIDDLE_LOWER = 11;
    private final int LEFT_BOTTOM = 12;

    private final int xAxis = 0;
    private final int yAxis = 1;

//==============================================================================
//============================== Constructor ===================================

    public ArcadeControls(int port) {
        this.joy = new Joystick(port);
    }

//==============================================================================
//============================ Create Buttons ==================================

    JoystickButton smallTop = new JoystickButton(joy, SMALL_TOP);
    JoystickButton smallBottom = new JoystickButton(joy, SMALL_BOTTOM);

    JoystickButton rightTop = new JoystickButton(joy, RIGHT_TOP);
    JoystickButton rightMiddleUpper = new JoystickButton(joy, RIGHT_MIDDLE_UPPER);
    JoystickButton rightMiddleLower = new JoystickButton(joy, RIGHT_MIDDLE_LOWER);
    JoystickButton rightBottom = new JoystickButton(joy, RIGHT_BOTTOM);

    JoystickButton leftTop = new JoystickButton(joy, LEFT_TOP);
    JoystickButton leftMiddleUpper = new JoystickButton(joy, LEFT_MIDDLE_UPPER);
    JoystickButton leftMiddleLower = new JoystickButton(joy, LEFT_MIDDLE_LOWER);
    JoystickButton leftBottom = new JoystickButton(joy, LEFT_BOTTOM);

//==============================================================================
//================================ Getters =====================================

    public Joystick getJoy() {
        return joy;
    }
    public JoystickButton getSmallTop() {
        return smallTop;
    }
    public JoystickButton getSmallBottom() {
        return smallBottom;
    }
    public JoystickButton getRightTop() {
        return rightTop;
    }
    public JoystickButton getRightMiddleUpper() {
        return rightMiddleUpper;
    }
    public JoystickButton getRightMiddleLower() {
        return rightMiddleLower;
    }
    public JoystickButton getRightBottom() {
        return rightBottom;
    }
    public JoystickButton getLeftTop() {
        return leftTop;
    }
    public JoystickButton getLeftMiddleUpper() {
        return leftMiddleUpper;
    }
    public JoystickButton getLeftMiddleLower() {
        return leftMiddleLower;
    }
    public JoystickButton getLeftBottom() {
        return leftBottom;
    }
    public double getX() {
        return joy.getRawAxis(xAxis);
    }
    public double getY() {
        return -joy.getRawAxis(yAxis);
    }


//==============================================================================
//============================== Functions =====================================
    public boolean isYAxisActive() {
        return getY() > 0.1 || getY() < -0.1;
    }

}
