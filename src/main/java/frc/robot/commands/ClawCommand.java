package frc.robot.commands;

import frc.robot.subsystems.ClawSubsystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ClawCommand extends CommandBase {
  private final ClawSubsystem claw;
  private final XboxController controller;

  /**
   * Creates a new ClawCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ClawCommand(ClawSubsystem subsystem, XboxController controller) {
    this.claw = subsystem;
    this.controller = controller;

    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    claw.setSpeed(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    claw.setSpeed(-controller.getRawAxis(3));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    claw.setSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
