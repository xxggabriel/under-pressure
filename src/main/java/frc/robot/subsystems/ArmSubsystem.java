package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ArmSubsystem extends SubsystemBase {

    private final Spark arm = new Spark(Constants.Arm.motor);

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
        arm.set(speed);
    }
}
