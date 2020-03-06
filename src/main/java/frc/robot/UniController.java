package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;

public class UniController {

    Joystick lStick, rStick;
    XboxController xbox;

    public UniController(XboxController x) {
        xbox = x;
        lStick = null;
        rStick = null;
    }

    public UniController(Joystick l, Joystick r) {
        lStick = l;
        rStick = r;
        xbox = null;
    }

    
}