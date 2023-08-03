package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.PneumaticSubsystem;

public class AutonomousCommand extends CommandBase {

  private DriveTrainSubsystem robotDrive;
  private ClawSubsystem robotClaw;
  private ArmSubsystem robotArm;
  private PneumaticSubsystem robotPneumatic;

  Timer m_timer = new Timer();
  double startTime;

  /**
   * Creates a new AutonomusCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public AutonomousCommand(DriveTrainSubsystem driveTrain, ClawSubsystem claw, ArmSubsystem arm, PneumaticSubsystem pneumatic) {
    this.robotDrive = driveTrain;
    this.robotClaw = claw;
    this.robotArm = arm;
    this.robotPneumatic = pneumatic;

    addRequirements(driveTrain, claw, arm, pneumatic);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.robotDrive.resetEncoders();
    m_timer.reset();
    m_timer.start();
    startTime = m_timer.getFPGATimestamp();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    System.out.println("Start: " + startTime);
    System.out.println("Timestamp: " + m_timer.getFPGATimestamp());
    if (m_timer.getFPGATimestamp() - startTime < 3.4) {
      this.robotArm.setSpeed(1);
    } else {
      this.robotArm.setSpeed(0);
      this.robotPneumatic.close();
      this.robotClaw.setSpeed(1);
      if (m_timer.getFPGATimestamp() - startTime < 4.2) {
      } else {
       andar();
      }
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    this.robotPneumatic.open();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  public void andar() {
    // Obtenha a distância atual percorrida pelos encoders
    double leftDistance = robotDrive.getLeftEncoder().get();
    double rightDistance = robotDrive.getRightEncoder().get();

    // Calcule a distância total percorrida (a média dos dois lados)
    double averageDistance = (leftDistance + rightDistance) / 2.0;
    averageDistance = averageDistance / 600;
    System.out.println("Andou: " + averageDistance);
    // Se a distância total ainda for menor que a distância desejada, continue
    // andando para frente
    if (averageDistance < 4) {
    // Ajuste aqui a velocidade e direção para o movimento do seu robô
    double speed = 0.6; // Exemplo: velocidade de 50%
    double rotation = 0.0; // Exemplo: sem rotação

    // Defina os motores de acordo com a velocidade e rotação
    robotDrive.arcadeDrive(speed, rotation);
    } else {
    // Caso contrário, pare o robô
    robotDrive.arcadeDrive(0, 0);
    }
  }
}
