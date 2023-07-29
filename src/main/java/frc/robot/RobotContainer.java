// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.ArmCommand;
import frc.robot.commands.ClawCommand;
import frc.robot.commands.DriveTrainCommand;
import frc.robot.commands.PneumaticCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.PneumaticSubsystem;

public class RobotContainer {
  private OI oi = new OI();

  private final DriveTrainSubsystem robotDrive = new DriveTrainSubsystem();
  private final ClawSubsystem robotClaw = new ClawSubsystem();
  private final ArmSubsystem robotArm = new ArmSubsystem();
  private final PneumaticSubsystem robotPneumatic = new PneumaticSubsystem();

  


  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    DriveTrainCommand driveCommand = new DriveTrainCommand(robotDrive, oi.getDriverController());
    robotDrive.setDefaultCommand(driveCommand);

    ClawCommand clawCommand = new ClawCommand(robotClaw, oi.getOperatorController());
    robotClaw.setDefaultCommand(clawCommand);

    PneumaticCommand pneumaticCommand = new PneumaticCommand(robotPneumatic, oi.getOperatorController());
    robotPneumatic.setDefaultCommand(pneumaticCommand);
    
    ArmCommand armCommand = new ArmCommand(robotArm, robotPneumatic, oi.getOperatorController());
    robotArm.setDefaultCommand(armCommand);
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
