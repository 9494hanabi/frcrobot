package frc.robot.subsystems.Swerve;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;

public class Module {
  private final SparkMax driveMotor;
  private final RelativeEncoder driveEncoder;
  private final SparkMax steerMotor;
  private final CANcoder steerEncoder;

  private final SimpleMotorFeedforward driveFeedforward;
  private final PIDController drivePID;
  private final PIDController steerPID;

  private static double gearRatio = 6.75;
  private static double wheelDiameterMeters = Units.inchesToMeters(3);
  private static double motorMaxRPM = 5676;

  private static double wheelMaxAngularVelocity = (motorMaxRPM / (60 * gearRatio)) / 2;
  private static double wheelMaxLinearVelocity = wheelMaxAngularVelocity * wheelDiameterMeters * Math.PI;

  public static double getGearRatio() {
    return gearRatio;
  }

  public static void setGearRatio(double ratio) {
    gearRatio = ratio;
    updateDerivedValues();
  }

  public static double getMotorMaxRPM() {
    return motorMaxRPM;
  }

  public static void setMotorMaxRPM(double rpm) {
    motorMaxRPM = rpm;
    updateDerivedValues();
  }

  public static double getwheelMaxAngularVelocity() {
    return wheelMaxAngularVelocity;
  }

  public static double getwheelMaxLinearVelocity() {
    return wheelMaxLinearVelocity;
  }

  public static void setwheelMaxLinearVelocity (double maxLinearVel) {
    wheelMaxLinearVelocity = maxLinearVel;
  }

  public static void updateDerivedValues() {
    wheelMaxAngularVelocity = (motorMaxRPM / (60 * gearRatio)) / 2;
    wheelMaxLinearVelocity = wheelMaxAngularVelocity * wheelDiameterMeters * Math.PI;
  }

  public Module(int id) {
    double steerEncoderOffset;

    switch (id) {
      case 0 -> { //左前
        driveMotor = new SparkMax(10, SparkLowLevel.MotorType.kBrushless);
        steerMotor = new SparkMax(11, SparkLowLevel.MotorType.kBrushless);
        steerEncoder = new CANcoder(12);
        steerEncoderOffset = 0.512;
      }
      case 1 -> { //右前
        driveMotor = new SparkMax(7, SparkLowLevel.MotorType.kBrushless);
        steerMotor = new SparkMax(8, SparkLowLevel.MotorType.kBrushless);
        steerEncoder = new CANcoder(9);
        steerEncoderOffset = 0.446;
      }
      case 2 -> { //左後ろ
        driveMotor = new SparkMax(4, SparkLowLevel.MotorType.kBrushless);
        steerMotor = new SparkMax(21, SparkLowLevel.MotorType.kBrushless);
        steerEncoder = new CANcoder(6);
        steerEncoderOffset = 0.313;
      }
      case 3 -> { //右後ろ
        driveMotor = new SparkMax(13, SparkLowLevel.MotorType.kBrushless);
        steerMotor = new SparkMax(2, SparkLowLevel.MotorType.kBrushless);
        steerEncoder = new CANcoder(3);
        steerEncoderOffset = 0.070;
      }
      default -> throw new IndexOutOfBoundsException("Invalid Module ID:" + id);
    }

    //ドライブモーターの設定
    SparkMaxConfig driveMotorConfig = new SparkMaxConfig();
    driveMotorConfig.idleMode(SparkBaseConfig.IdleMode.kBrake);
    driveMotorConfig.smartCurrentLimit(45);
    driveMotorConfig.encoder.positionConversionFactor(wheelDiameterMeters * Math.PI / gearRatio);
    driveMotorConfig.encoder.velocityConversionFactor(wheelDiameterMeters * Math.PI /(gearRatio * 60));
    driveEncoder = driveMotor.getEncoder();
    driveMotor.configure(driveMotorConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kNoPersistParameters);

    //ステアモーターの設定
    SparkMaxConfig steerMotorConfig = new SparkMaxConfig();
    steerMotorConfig.idleMode(SparkBaseConfig.IdleMode.kBrake);
    steerMotorConfig.smartCurrentLimit(25);
    steerMotor.configure(steerMotorConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kNoPersistParameters);

    //ステアエンコーダの設定
    CANcoderConfiguration steerEncoderConfig = new CANcoderConfiguration();
    steerEncoderConfig.MagnetSensor.MagnetOffset = steerEncoderOffset;
    steerEncoder.getConfigurator().apply(steerEncoderConfig);

    driveFeedforward = new SimpleMotorFeedforward(0, 12 / wheelMaxLinearVelocity);
    drivePID = new PIDController(2 / wheelMaxLinearVelocity, 0, 0);
    steerPID = new PIDController(5, 0, 0.2);
    steerPID.enableContinuousInput(-0.5, 0.5);
  }

  public SwerveModulePosition getPosition() {
    double distance = driveEncoder.getPosition();
    Rotation2d angle = new Rotation2d(steerEncoder.getAbsolutePosition().getValueAsDouble());
    return new SwerveModulePosition(distance, angle);
  }

  void run(SwerveModuleState desiredState) {
    double currentSteerAngleRotations = steerEncoder.getAbsolutePosition().getValueAsDouble();
    double currentMotorVelMeterPerSec = driveEncoder.getVelocity();
    desiredState.optimize(Rotation2d.fromRotations(currentSteerAngleRotations));
    driveMotor.setVoltage(
      driveFeedforward.calculate(desiredState.speedMetersPerSecond) + drivePID.calculate(currentMotorVelMeterPerSec, desiredState.speedMetersPerSecond));
    steerMotor.setVoltage(steerPID.calculate(currentSteerAngleRotations, desiredState.angle.getRotations()));
  }

  public double getDistanceMeters() {
    return driveEncoder.getPosition();
  }

  public Rotation2d getAngle() {
    return Rotation2d.fromRotations(steerEncoder.getAbsolutePosition().getValueAsDouble());
  }
}