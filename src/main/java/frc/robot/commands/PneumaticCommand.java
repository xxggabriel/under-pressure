package frc.robot.commands;

import frc.robot.subsystems.PneumaticSubsystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class PneumaticCommand extends CommandBase {
    private final PneumaticSubsystem m_subsystem;
    private final XboxController controller;

    /**
     * Creates a new PneumaticCommand.
     *
     * @param subsystem The subsystem used by this command.
     */
    public PneumaticCommand(PneumaticSubsystem subsystem, XboxController controller) {
        m_subsystem = subsystem;
        this.controller = controller;

        addRequirements(subsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (controller.getAButtonPressed()) {
            m_subsystem.open();
        } else if(controller.getBButtonPressed()) {
            m_subsystem.close();
        }
        
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        // m_subsystem.close();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
