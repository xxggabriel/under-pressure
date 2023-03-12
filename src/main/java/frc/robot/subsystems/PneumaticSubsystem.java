package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class PneumaticSubsystem extends SubsystemBase {

    private DoubleSolenoid solenoid;

    public PneumaticSubsystem() {
        this.solenoid = new DoubleSolenoid(
            Constants.OIConstants.pneumaticsController, 
            Constants.OIConstants.pneumaticsModuleType, 
            Constants.OIConstants.forwardChannel, 
            Constants.OIConstants.reverseChannel
        );
    }

    public void open() {
        solenoid.set(Value.kForward);
    }

    public void close() {
        solenoid.set(Value.kReverse);
    }

    public void controlWithJoystick(Joystick joystick, int openButton, int closeButton) {
        if (joystick.getRawButton(openButton)) {
            open();
        } else if (joystick.getRawButton(closeButton)) {
            close();
        }
    }

}
