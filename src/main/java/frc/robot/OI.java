package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;


public class OI {

    // Declaração do controle do operador
    private XboxController operatorController;
    private Joystick joystick;


    public OI() {
        operatorController = new XboxController(Constants.IO.Controller.port);
        joystick = new Joystick(Constants.IO.Joystick.port);
    }

    public XboxController getOperatorController()
    {
        return operatorController;
    }

    public Joystick getDriverController()
    {
        return joystick;
    }
}
