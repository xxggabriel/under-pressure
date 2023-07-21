package frc.robot.commands;

import frc.robot.subsystems.PneumaticSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class PneumaticCommand extends CommandBase {
    private final PneumaticSubsystem m_subsystem;

    /**
     * Creates a new PneumaticCommand.
     *
     * @param subsystem The subsystem used by this command.
     */
    public PneumaticCommand(PneumaticSubsystem subsystem) {
        m_subsystem = subsystem;

        addRequirements(subsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_subsystem.close();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_subsystem.close();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
