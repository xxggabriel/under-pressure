package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ArmSubsystem extends SubsystemBase {

    private final CANSparkMax arm = new CANSparkMax(Constants.Arm.motor, MotorType.kBrushed);

    /** Creates a new ArmSubsystem. */
    public ArmSubsystem() {

    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }

    public void setSpeed(double speed) {
        arm.set(speed * .8);
    }
}
