package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.DriveTrainSubsystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class DriveTrainCommand extends CommandBase {
    private final DriveTrainSubsystem driveTrain;
    private final Joystick joystick;

    /**
     * Creates a new DriveTrainCommand.
     *
     * @param subsystem The subsystem used by this command.
     */
    public DriveTrainCommand(DriveTrainSubsystem subsystem, Joystick joystick) {
        this.driveTrain = subsystem;
        this.joystick = joystick;

        addRequirements(this.driveTrain);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        // this.driveTrain.arcadeDrive(0, 0);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        double speed = this.joystick.getRawAxis(Constants.IO.Joystick.portSpeed);
        double rotation = this.joystick.getRawAxis(Constants.IO.Joystick.portRotation);

        // speed = speed * joystick.getRawAxis(3);

        // if (speed < .3) {
        //     speed = 0.3;
        // }

        this.driveTrain.arcadeDrive(speed, -rotation);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        // this.driveTrain.arcadeDrive(0, 0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
