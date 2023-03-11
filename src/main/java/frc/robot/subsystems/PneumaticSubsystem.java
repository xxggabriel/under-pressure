package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PneumaticSubsystem extends SubsystemBase {

    private Solenoid solenoid;

    public PneumaticSubsystem(int channel) {
        Solenoid solenoid = new Solenoid(PneumaticsModuleType.CTREPCM, 1);
    }

    public void open() {
        solenoid.set(true);
    }

    public void close() {
        solenoid.set(false);
    }

    public void controlWithJoystick(Joystick joystick, int openButton, int closeButton) {
        if (joystick.getRawButton(openButton)) {
            open();
        } else if (joystick.getRawButton(closeButton)) {
            close();
        }
    }

}
