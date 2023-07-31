// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.DriveTrainSubsystem;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  @Override
  public void robotInit() {
    m_robotContainer = new RobotContainer();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
  }

  @Override
  public void disabledExit() {
  }

  @Override
  public void autonomousInit() {
    ArmSubsystem armSubsystem = new ArmSubsystem();
    ClawSubsystem clawSubsystem = new ClawSubsystem();
    DriveTrainSubsystem driveTrainSubsystem = new DriveTrainSubsystem();

    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }

    // Obtenha a distância atual percorrida pelos encoders
    double leftDistance = driveTrainSubsystem.getLeftEncoder().getDistance();
    double rightDistance = driveTrainSubsystem.getRightEncoder().getDistance();

    // Calcule a distância total percorrida (a média dos dois lados)
    double averageDistance = (leftDistance + rightDistance) / 2.0;

    // Se a distância total ainda for menor que a distância desejada, continue
    // andando para frente
    if (averageDistance < 3) {
      // Ajuste aqui a velocidade e direção para o movimento do seu robô
      double speed = 0.5; // Exemplo: velocidade de 50%
      double rotation = 0.0; // Exemplo: sem rotação

      // Defina os motores de acordo com a velocidade e rotação
      leftMotor.set(speed + rotation);
      rightMotor.set(speed - rotation);

      driveTrainSubsystem.arcadeDrive(.6, 0);
    } else {
      // Caso contrário, pare o robô
      leftMotor.set(0.0);
      rightMotor.set(0.0);
    }
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void autonomousExit() {
  }

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void teleopExit() {
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {
  }

  @Override
  public void testExit() {
  }
}
