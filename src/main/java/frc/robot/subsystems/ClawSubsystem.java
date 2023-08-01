package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClawSubsystem extends SubsystemBase {

    private final CANSparkMax claw = new CANSparkMax(Constants.Craw.motor, MotorType.kBrushless);

    /** Creates a new ClawSubsystem. */
    public ClawSubsystem() {

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
        claw.set(speed);
    }
}
