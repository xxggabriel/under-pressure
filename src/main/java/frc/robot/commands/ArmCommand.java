package frc.robot.commands;

import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.PneumaticSubsystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ArmCommand extends CommandBase {
    private final ArmSubsystem armSubsystem;
    // private final PneumaticSubsystem pneumaticSubsystem;
    private final XboxController controller;

    /**
     * Creates a new ArmCommand.
     *
     * @param subsystem The subsystem used by this command.
     */
    public ArmCommand(ArmSubsystem armSubsystem, PneumaticSubsystem pneumaticSubsystem ,XboxController controller) {
        this.armSubsystem = armSubsystem;
        // this.pneumaticSubsystem = pneumaticSubsystem;
        this.controller = controller;

        addRequirements(this.armSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        armSubsystem.setSpeed(0);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        armSubsystem.setSpeed(1);
        // if (controller.getBButtonPressed()) {
        // } else if(controller.getYButtonPressed()) {
        //     armSubsystem.setSpeed(-1);
        // }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        armSubsystem.setSpeed(0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
