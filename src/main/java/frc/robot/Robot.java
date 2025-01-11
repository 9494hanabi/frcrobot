// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.hal.simulation.RoboRioDataJNI;
import edu.wpi.first.math.controller.PIDController;

public class Robot extends TimedRobot {

  // =====================================================
  // もともと "1つのモジュール" 用に書かれていた変数を
  // 4セット分複製して用意 (Module1〜Module4)
  // =====================================================

  // ----------------------
  // 共通: Joystick
  // ----------------------
  Joystick joystick = new Joystick(0);



  // ----------------------
  // Module1
  // ----------------------
  double position1 = 0;                      // (0.0~1.0)
  CANcoder encoder1 = new CANcoder(9);       // CANCoder(9)
  StatusSignal<Double> positionSignal1 = encoder1.getAbsolutePosition();
  private com.revrobotics.CANSparkMax turnMotor1;  // SPARK MAX for Module1
  private final PIDController m_pid1 = new PIDController(1.0, 0.0, 0.0);
  private com.revrobotics.CANSparkMax driveMotor1; // Module1用ドライブモーター

  // ----------------------
  // Module2
  // ----------------------
  double position2 = 0;
  CANcoder encoder2 = new CANcoder(6);
  StatusSignal<Double> positionSignal2 = encoder2.getAbsolutePosition();
  private com.revrobotics.CANSparkMax turnMotor2;
  private final PIDController m_pid2 = new PIDController(1.0, 0.0, 0.0);
  private com.revrobotics.CANSparkMax driveMotor2; // Module1用ドライブモーター

  // ----------------------
  // Module3
  // ----------------------
  double position3 = 0;
  CANcoder encoder3 = new CANcoder(3);
  StatusSignal<Double> positionSignal3 = encoder3.getAbsolutePosition();
  private com.revrobotics.CANSparkMax turnMotor3;
  private final PIDController m_pid3 = new PIDController(1.0, 0.0, 0.0);
  private com.revrobotics.CANSparkMax driveMotor3; // Module1用ドライブモーター

  // ----------------------
  // Module4
  // ----------------------
  double position4 = 0;
  CANcoder encoder4 = new CANcoder(12);
  StatusSignal<Double> positionSignal4 = encoder4.getAbsolutePosition();
  private com.revrobotics.CANSparkMax turnMotor4;
  private final PIDController m_pid4 = new PIDController(1.0, 0.0, 0.0);
  private com.revrobotics.CANSparkMax driveMotor4; // Module1用ドライブモーター
//ここの値はいろいろかえておｋ

  // デッドゾーン (4モジュール共通)
  double deadzone = 0.05;

  private com.revrobotics.CANSparkMax initializeMotor(int id) {
    com.revrobotics.CANSparkMax motor = new com.revrobotics.CANSparkMax(
      id,
      com.revrobotics.CANSparkMaxLowLevel.MotorType.kBrushless
    );
    motor.restoreFactoryDefaults();
    return motor;
}



  // =====================================================
  // robotInit
  // =====================================================
  @Override
  public void robotInit() {

    // ==================================================
    // Module1: 複製元コードと同じ処理
    // ==================================================
    encoder1 = new CANcoder(9, "rio");
    {
      CANcoderConfiguration config = new CANcoderConfiguration();
      MagnetSensorConfigs magnetSensorConfigs = new MagnetSensorConfigs();

      magnetSensorConfigs.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;
      magnetSensorConfigs.MagnetOffset = 45.0;
      magnetSensorConfigs.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
      //magnetSensorConfigs.MagnetOffset = 1.0; // スケーリング

      config.MagnetSensor = magnetSensorConfigs;

      encoder1.getConfigurator().apply(config);
    }
    position1 = encoder1.getAbsolutePosition().getValue();

    turnMotor1 = new com.revrobotics.CANSparkMax(
      8, // ← 仮ID (後で変更可能)
      com.revrobotics.CANSparkMaxLowLevel.MotorType.kBrushless
    );
    turnMotor1.restoreFactoryDefaults();
    driveMotor1 = initializeMotor(7); // ドライブ用

    m_pid1.setTolerance(0.01);
    m_pid1.enableContinuousInput(0.0, 1.0);


    // ==================================================
    // Module2: 完全に同じロジックを複製
    // ==================================================
    encoder2 = new CANcoder(6, "rio");
    {
      CANcoderConfiguration config = new CANcoderConfiguration();
      MagnetSensorConfigs magnetSensorConfigs = new MagnetSensorConfigs();

      magnetSensorConfigs.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;
      magnetSensorConfigs.MagnetOffset = 45.0;
      magnetSensorConfigs.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
      //magnetSensorConfigs.MagnetOffset = 1.0; // スケーリング

      config.MagnetSensor = magnetSensorConfigs;

      encoder2.getConfigurator().apply(config);
    }
    position2 = encoder2.getAbsolutePosition().getValue();

    turnMotor2 = new com.revrobotics.CANSparkMax(
      5, // ← 仮ID
      com.revrobotics.CANSparkMaxLowLevel.MotorType.kBrushless
    );
    turnMotor2.restoreFactoryDefaults();
    driveMotor2 = initializeMotor(4); // ドライブ用

    m_pid2.setTolerance(0.01);
    m_pid2.enableContinuousInput(0.0, 1.0);


    // ==================================================
    // Module3
    // ==================================================
    encoder3 = new CANcoder(3, "rio");
    {
      CANcoderConfiguration config = new CANcoderConfiguration();
      MagnetSensorConfigs magnetSensorConfigs = new MagnetSensorConfigs();

      magnetSensorConfigs.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;
      magnetSensorConfigs.MagnetOffset = 45.0;
      magnetSensorConfigs.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
      //magnetSensorConfigs.MagnetOffset = 1.0; // スケーリング
      
      config.MagnetSensor = magnetSensorConfigs;

      encoder3.getConfigurator().apply(config);
    }
    position3 = encoder3.getAbsolutePosition().getValue()-0.184;

    turnMotor3 = new com.revrobotics.CANSparkMax(
      2, // 仮ID
      com.revrobotics.CANSparkMaxLowLevel.MotorType.kBrushless
    );
    turnMotor3.restoreFactoryDefaults();
    driveMotor3 = initializeMotor(13); // ドライブ用

    m_pid3.setTolerance(0.01);
    m_pid3.enableContinuousInput(0.0, 1.0);


    // ==================================================
    // Module4
    // ==================================================
    encoder4 = new CANcoder(12, "rio");
    {
      CANcoderConfiguration config = new CANcoderConfiguration();
      MagnetSensorConfigs magnetSensorConfigs = new MagnetSensorConfigs();

      magnetSensorConfigs.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;
      magnetSensorConfigs.MagnetOffset = 45.0;
      magnetSensorConfigs.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
      //magnetSensorConfigs.MagnetOffset = 1.0; // スケーリング

      config.MagnetSensor = magnetSensorConfigs;

      encoder4.getConfigurator().apply(config);
      
    }
    position4 = encoder4.getAbsolutePosition().getValue()-0.235;

    turnMotor4 = new com.revrobotics.CANSparkMax(
      11, // 仮ID
      com.revrobotics.CANSparkMaxLowLevel.MotorType.kBrushless
    );
    turnMotor4.restoreFactoryDefaults();
    driveMotor4 = initializeMotor(10); // ドライブ用

    m_pid4.setTolerance(0.01);
    m_pid4.enableContinuousInput(0.0, 1.0);


    // position1 -= 0.301;
    // position2 += 0.051;
    // position3 -= 0.184;
    // position4 -= 0.235;
  }


  // =====================================================
  // robotPeriodic
  //   (複製元コードでは position%1.0 していたが、
  //    今回は複数モジュール分に応じて行う)
  // =====================================================
  @Override
  public void robotPeriodic() {

    // ===== Module1
    position1 = position1 % 1.0;
    if (position1 < 0.0) {
      position1 += 1.0;
    } else if (position1 > 1.0) {
      position1 -= 1.0;
    }
    SmartDashboard.putNumber("CAN Encoder1", position1);

    // ===== Module2
    position2 = position2 % 1.0;
    if (position2 < 0.0) {
      position2 += 1.0;
    } else if (position2 > 1.0) {
      position2 -= 1.0;
    }
    SmartDashboard.putNumber("CAN Encoder2", position2);

    // ===== Module3
    position3 = position3 % 1.0;
    if (position3 < 0.0) {
      position3 += 1.0;
    } else if (position3 > 1.0) {
      position3 -= 1.0;
    }
    SmartDashboard.putNumber("CAN Encoder3", position3);

    // ===== Module4
    position4 = position4 % 1.0;
    if (position4 < 0.0) {
      position4 += 1.0;
    } else if (position4 > 1.0) {
      position4 -= 1.0;
    }
    SmartDashboard.putNumber("CAN Encoder4", position4);

  }


  // =====================================================
  // teleopPeriodic
  //   (複製元コードと同じロジックをモジュールごとに)
  // =====================================================
  @Override
  public void teleopPeriodic() {


    //robot size
    double  RobotWith = 6.75;
    double  RobotLength = 6.75;
    double  R =  Math.sqrt( RobotLength*RobotLength + RobotWith * RobotWith);

    // ----------------------------
    // ① Joystick の X軸を取得 + デッドゾーン
    // ----------------------------
    double X = joystick.getRawAxis(0);
    if (Math.abs(X) < deadzone) {
      X = 0.0;
    }
    double Y = -joystick.getRawAxis(1);
    if (Math.abs(Y) < deadzone) {
      Y = 0.0;
    }
    double Rotate = -joystick.getRawAxis(4);
    if (Math.abs(Rotate) < deadzone) {
      Rotate = 0.0;
    

    //べくとるせってい
    double A = X - Rotate * (RobotWith / R);
    double B = X + Rotate * (RobotWith / R);
    double C = Y - Rotate * (RobotLength / R);
    double D = Y + Rotate * (RobotLength / R);

    //module1
    double FRspeed = Math.sqrt(B*B + C*C);
    double FRrad = Math.toDegrees(Math.atan2(B,C));
    //module2
    double BLspeed = Math.sqrt(A*A+ D*D);
    double BLrad = Math.toDegrees(Math.atan2(A,D));
    //module3
    double BRspeed = Math.sqrt(A*A + C*C);
    double BRrad = Math.toDegrees(Math.atan2(A,C));
    //module4
    double FLspeed = Math.sqrt(B*B + D*D);
    double FLrad = Math.toDegrees(Math.atan2(B,D));

    // ----------------------------
    // ③ targetPos(角度) は sqrt(x^2 + y^2) を 0～1 にクランプ
    // ----------------------------
    // double rawAngle = Math.sqrt(X * X + Y * Y); // 0.0 ～ sqrt(2) くらい
    // //double targetPos = rawAngle;
    // double driveSpeed = rawAngle * 0.3;
    // 速度を 0〜1 に正規化

    double maxSpeed = Math.max(
      Math.max(FLspeed, FRspeed),
      Math.max(BLspeed, BRspeed)
    );
    if (maxSpeed > 1.0) {
      FLspeed /= maxSpeed;
      FRspeed /= maxSpeed;
      BLspeed /= maxSpeed;
      BRspeed /= maxSpeed;
    }


    //-----------------
    //角度コントロール編
    //-----------------
    // double angleRad = Math.atan2(Y,X);
    // double angleDeg = Math.toDegrees(angleRad);

    //角度の正規化
    double desiredAngleDeg1 = FRspeed;
    double desiredAngleDeg2 = BLspeed;
    double desiredAngleDeg3 = BRspeed;
    double desiredAngleDeg4 = FLspeed;

    if (desiredAngleDeg1 < 0) {
      desiredAngleDeg1 += 360.0;
    }else if (desiredAngleDeg1 >= 360.0){
      desiredAngleDeg1 -= 360.0;
    }

        if (desiredAngleDeg2 < 0) {
      desiredAngleDeg2 += 360.0;
    }else if (desiredAngleDeg2 >= 360.0){
      desiredAngleDeg2 -= 360.0;
    }

        if (desiredAngleDeg3 < 0) {
      desiredAngleDeg3 += 360.0;
    }else if (desiredAngleDeg3 >= 360.0){
      desiredAngleDeg3 -= 360.0;
    }

        if (desiredAngleDeg4 < 0) {
      desiredAngleDeg4 += 360.0;
    }else if (desiredAngleDeg4 >= 360.0){
      desiredAngleDeg4 -= 360.0;
    }

    double targetPos1 = desiredAngleDeg1 / 360.0;
    double targetPos2 = desiredAngleDeg2 / 360.0;
    double targetPos3 = desiredAngleDeg3 / 360.0;
    double targetPos4 = desiredAngleDeg4 / 360.0;
    //おわり

    // ==================================================
    // Module1: (複製のまま)
    // ==================================================
    positionSignal1.refresh();
    position1 = positionSignal1.getValue() -0.301;
    

    double output1 = m_pid1.calculate(position1, targetPos1);
    //output1 = clamp(output1, -1.0, 1.0);
    turnMotor1.set(output1);
    driveMotor1.set(FRspeed); // ドライブモーターに速度を適
 

    SmartDashboard.putNumber("TargetPos1", targetPos1);
    SmartDashboard.putNumber("PID Output1", output1);


    // ==================================================
    // Module2
    // ==================================================
    positionSignal2.refresh();
    position2 = positionSignal2.getValue() +0.051;

    

    

    double output2 = m_pid2.calculate(position2, targetPos2);
    //output2 = clamp(output2, -1.0, 1.0);
    turnMotor2.set(output2);
    driveMotor2.set(BLspeed); 

    SmartDashboard.putNumber("TargetPos2", targetPos2);
    SmartDashboard.putNumber("PID Output2", output2);


    // ==================================================
    // Module3
    // ==================================================
    positionSignal3.refresh();
    position3 = positionSignal3.getValue() -0.181;

    
   

    double output3 = m_pid3.calculate(position3, targetPos3);
    //output3 = clamp(output3, -1.0, 1.0);
    turnMotor3.set(output3);
    driveMotor3.set(BRspeed); 

    SmartDashboard.putNumber("TargetPos3", targetPos3);
    SmartDashboard.putNumber("PID Output3", output3);


    // ==================================================
    // Module4
    // ==================================================
    positionSignal4.refresh();
    position4 = positionSignal4.getValue() -0.235;

    
    

    double output4 = m_pid4.calculate(position4, targetPos4) + Rotate;
    //output4 = clamp(output4, -1.0, 1.0);
    turnMotor4.set(output4);
    driveMotor4.set(FLspeed); 

    SmartDashboard.putNumber("TargetPos4", targetPos4);
    SmartDashboard.putNumber("PID Output4", output4);
  }


  // =====================================================
  // clamp メソッド (そのまま複製)
  // =====================================================
  // private double clamp(double val, double min, double max) {
  //   return Math.max(min, Math.min(val, max));
  // }


  // === 以下、他のモードは既存のまま ===

  @Override
  public void autonomousInit() {}
  @Override
  public void autonomousPeriodic() {}
  @Override
  public void disabledInit() {}
  @Override
  public void disabledPeriodic() {}
  @Override
  public void testInit() {}
  @Override
  public void testPeriodic() {}
  @Override
  public void simulationInit() {}
  @Override
  public void simulationPeriodic() {}
}
}

// わっしょいはなび 関数作りたい