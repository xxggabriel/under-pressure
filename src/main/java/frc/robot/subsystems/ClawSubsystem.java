package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClawSubsystem extends SubsystemBase {

    private final Spark claw = new Spark(Constants.Craw.motor);

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
