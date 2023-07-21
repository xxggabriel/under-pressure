package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveTrainSubsystem extends SubsystemBase {

    private final Spark leftMotor1 = new Spark(Constants.DriveTrain.leftFrontMotor);
    private final VictorSP leftMotor2 = new VictorSP(Constants.DriveTrain.leftRearMotor);

    private final Spark rightMotor1 = new Spark(Constants.DriveTrain.rightFrontMotor);
    private final VictorSP rightMotor2 = new VictorSP(Constants.DriveTrain.rightRearMotor);

    private final MotorControllerGroup leftMotors = new MotorControllerGroup(leftMotor1, leftMotor2);
    private final MotorControllerGroup rightMotors = new MotorControllerGroup(rightMotor1, rightMotor2);

    private final DifferentialDrive drive = new DifferentialDrive(leftMotors, rightMotors);

    private final Encoder leftEncoder = new Encoder(
            Constants.Sensor.leftEncoderPort1,
            Constants.Sensor.leftEncoderPort2,
            Constants.Sensor.leftEncoderReversed);

    private final Encoder rightEncoder = new Encoder(
            Constants.Sensor.rightEncoderPort1,
            Constants.Sensor.rightEncoderPort2,
            Constants.Sensor.rightEncoderReversed);

    private final AnalogGyro gyro = new AnalogGyro(Constants.Sensor.gyroPort);
    private DifferentialDriveOdometry odometry = new DifferentialDriveOdometry(gyro.getRotation2d(), 0, 0);

    /** Creates a new DriveTrain. */
    public DriveTrainSubsystem() {
        rightMotors.setInverted(Constants.DriveTrain.rightReversed);
        leftMotors.setInverted(Constants.DriveTrain.leftReversed);

        // Sets the distance per pulse for the encoders
        leftEncoder.setDistancePerPulse(Constants.Sensor.encoderDistancePerPulse);
        rightEncoder.setDistancePerPulse(Constants.Sensor.encoderDistancePerPulse);

        drive.feed();

        resetEncoders();
    }

    @Override
    public void periodic() {
        updateOdometry();
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }

    public void arcadeDrive(double xSpeed, double zRotation) {
        drive.arcadeDrive(xSpeed, zRotation);
        // Creating my kinematics object using the wheel locations.
        MecanumDriveKinematics m_kinematics = new MecanumDriveKinematics(
            getPose().getTranslation(),getPose().getTranslation(),getPose().getTranslation(),getPose().getTranslation());
        // Enviar informações do chassi para o Shuffleboard
        // Shuffleboard.getTab("Chassi").addNumber("Velocidade Esquerda", leftMotors.get());
        // Shuffleboard.getTab("Chassi").addNumber("Velocidade Direita", rightMotors.get());
    }

    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }

    public void updateOdometry() {
        double left = leftEncoder.getDistance();
        double right = rightEncoder.getDistance();
        odometry.update(gyro.getRotation2d(), left, right);
    }

    public void resetEncoders() {
        leftEncoder.reset();
        rightEncoder.reset();
    }
}
