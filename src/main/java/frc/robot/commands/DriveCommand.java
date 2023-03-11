// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class DriveCommand extends CommandBase {

  DriveSubsystem driveSubsystem;
  Joystick joystick;

  /** Creates a new DriveCommand. */
  public DriveCommand(DriveSubsystem driveSubsystem, Joystick joystick) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveSubsystem);

    this.driveSubsystem = driveSubsystem;
    this.joystick = joystick;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    driveSubsystem.arcadeDrive(
        -joystick.getRawAxis(1),
        joystick.getRawAxis(0));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
