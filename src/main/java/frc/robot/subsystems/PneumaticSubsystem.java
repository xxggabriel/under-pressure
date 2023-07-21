package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class PneumaticSubsystem extends SubsystemBase {
    private DoubleSolenoid solenoid;

    public PneumaticSubsystem() {
        solenoid = new DoubleSolenoid(
                Constants.Pneumatic.controller,
                Constants.Pneumatic.moduleType,
                Constants.Pneumatic.forwardChannel,
                Constants.Pneumatic.reverseChannel);
    }

    public void open() {
        solenoid.set(Value.kForward);
    }

    public void close() {
        solenoid.set(Value.kReverse);
    }
}
