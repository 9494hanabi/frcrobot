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
// import com.kauailabs.navx.frc.;
import edu.wpi.first.units.measure.Distance;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage; 
import com.ctre.phoenix6.signals.NeutralModeValue;

class Module {
    private final SparkMax driveMotor;
    private final RelativeEncoder driveEncoder;
    private final SparkMax steerMotor;
    private final CANcoder steerEncoder;
  
    private final SimpleMotorFeedforward driveFeedforward;
    private final PIDController drivePID;
    private final PIDController steerPID;
    public Distance getDistanceMeters;

    private TalonFX krakenMotorR, krakenMotorL;
    private Object Elevator;
      
    
      // モジュールのギア比
      // TODO: セットする
      private static final double gearRatio = 6.75;
      // ホイールの直径
      private static final double wheelDiameterMeters = Units.inchesToMeters(3);
      // NEOの最大RPM
      private static final double motorMaxRPM = 2500;
      // ホイールの最大角速度。秒速回転。
      private static final double wheelMaxAngularVelocity = (motorMaxRPM / (60 * gearRatio)) / 2;
      // ホイールの最大線速度。秒速メートル。
      static final double wheelMaxLinearVelocity =
          wheelMaxAngularVelocity * wheelDiameterMeters * Math.PI;
    
      //private static final String Elevator = null;
          
            // 最大ボルテージ / 最大速度 = ボルト / 秒速メートル。すなわち、秒速1メートルのスピードだすために何ボルト必要か
          
            public Module(int id) {
              // エンコーダーのオフセット。0から360じゃなくて-0.5から+0.5。
              // 真っ直ぐ前にセットしてPhoenix Tunerでオフセットを測る
              double steerEncoderOffset;
          
              switch (id) {
                case 0 -> { // 左前
                  driveMotor = new SparkMax(10, SparkLowLevel.MotorType.kBrushless); // TODO: CAN IDをセットする
                  steerMotor = new SparkMax(11, SparkLowLevel.MotorType.kBrushless); // TODO: CAN IDをセットする
                  steerEncoder = new CANcoder(12); // TODO: CAN IDをセットする
                  steerEncoderOffset = 0.512; // TODO: エンコーダーのオフセットをセットする
                }
                case 1 -> { // 右前
                  driveMotor = new SparkMax(7, SparkLowLevel.MotorType.kBrushless); // TODO: CAN IDをセットする
                  steerMotor = new SparkMax(8, SparkLowLevel.MotorType.kBrushless); // TODO: CAN IDをセットする
                  steerEncoder = new CANcoder(9); // TODO: CAN IDをセットする
                  steerEncoderOffset = 0.446; // TODO: エンコーダーのオフセットをセットする
                }
                case 2 -> { // 左後ろ
                  driveMotor = new SparkMax(4, SparkLowLevel.MotorType.kBrushless); // TODO: CAN IDをセットする
                  steerMotor = new SparkMax(5, SparkLowLevel.MotorType.kBrushless); // TODO: CAN IDをセットする
                  steerEncoder = new CANcoder(6); // TODO: CAN IDをセットする
                  steerEncoderOffset = 0.313; // TODO: エンコーダーのオフセットをセットする
                }
                case 3 -> { // 右後ろ
                  driveMotor = new SparkMax(13, SparkLowLevel.MotorType.kBrushless); // TODO: CAN IDをセットする
                  steerMotor = new SparkMax(2, SparkLowLevel.MotorType.kBrushless); // TODO: CAN IDをセットする
                  steerEncoder = new CANcoder(3); // TODO: CAN IDをセットする
                  steerEncoderOffset = 0.070; // TODO: エンコーダーのオフセットをセットする
                }
                default -> throw new IndexOutOfBoundsException();
              }

    // ドライブモーターのコンフィグ
    SparkMaxConfig driveMotorConfig = new SparkMaxConfig();
    driveMotorConfig.idleMode(SparkBaseConfig.IdleMode.kBrake); // 動かない時にブレーキかける
    driveMotorConfig.smartCurrentLimit(45); // モーターの電流の制御 (6０アンペアマックス)
    // モーターの回転からホイールのメートル距離への変換
    driveMotorConfig.encoder.positionConversionFactor(wheelDiameterMeters * Math.PI / gearRatio);
    // モーターのRPMからホイールの秒速メートルへの変換
    driveMotorConfig.encoder.velocityConversionFactor(
        wheelDiameterMeters * Math.PI / (gearRatio * 60));

    // ドライブモーターのセンサー
    driveEncoder = driveMotor.getEncoder();

    // コンフィグをセットする
    driveMotor.configure(
        driveMotorConfig,
        SparkBase.ResetMode.kResetSafeParameters,
        SparkBase.PersistMode.kNoPersistParameters);

    SparkMaxConfig steerMotorConfig = new SparkMaxConfig();
    steerMotorConfig.idleMode(SparkBaseConfig.IdleMode.kBrake);
    // NEO 550は燃えやすいから電流制御結構低くないとダメなの。
    // 30アンペアでも高いかも。d
    steerMotorConfig.smartCurrentLimit(25);

    steerMotor.configure(
        steerMotorConfig,
        SparkBase.ResetMode.kResetSafeParameters,
        SparkBase.PersistMode.kNoPersistParameters);

    CANcoderConfiguration steerEncoderConfig = new CANcoderConfiguration();
    steerEncoderConfig.MagnetSensor.MagnetOffset = steerEncoderOffset;
    steerEncoder.getConfigurator().apply(steerEncoderConfig);

    driveFeedforward = new SimpleMotorFeedforward(0, 12 / wheelMaxLinearVelocity);
    // ホイールがターゲットの速度に行くためのフィードバック
    drivePID = new PIDController(2 / wheelMaxLinearVelocity, 0, 0);
    // ホイールがターゲットの角度に行くためのフィードバック
    steerPID = new PIDController(48, 0, 0.1);
    // +180度と-180度は同じだからPIDでそうセットする
    steerPID.enableContinuousInput(-.5, .5);
  
  }

  public SwerveModulePosition getPosition() {
    double distance = driveEncoder.getPosition();
    Rotation2d angle = new Rotation2d(steerEncoder.getAbsolutePosition().getValueAsDouble());
    return new SwerveModulePosition(distance, angle);
  }

  void run(SwerveModuleState desiredState) {
    // センサーからデータゲット
    double currentSteerAngleRotations = steerEncoder.getAbsolutePosition().getValueAsDouble();
    double currentMotorVelMetersPerSec = driveEncoder.getVelocity();

    // たまにはターゲットの速度とターゲットの角度をひっくり返すほうが速い
    // こうすればホイールの回転は最大で90度
    desiredState.optimize(Rotation2d.fromRotations(currentSteerAngleRotations));

    driveMotor.setVoltage(
        // フィードフォワードで必要なボルテージ予測し、フィードバックでターゲットからの逸脱に反応する
        driveFeedforward.calculate(desiredState.speedMetersPerSecond)
            + drivePID.calculate(
                currentMotorVelMetersPerSec, // 今の速度
                desiredState.speedMetersPerSecond)); // ターゲットの速度
    steerMotor.setVoltage(
        // フィードバックでターゲットからの逸脱に反応する
        steerPID.calculate(
            currentSteerAngleRotations, // 今のポジション
            desiredState.angle.getRotations())); // ターゲットのポジション
  }
  public double getDistanceMeters() {
    return driveEncoder.getPosition();
  }

  public Rotation2d getAngle() {
    return Rotation2d.fromRotations(steerEncoder.getAbsolutePosition().getValueAsDouble());
  }

}
