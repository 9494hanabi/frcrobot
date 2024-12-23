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
  StatusSignal<Double> positionSignal1 = encoder1.getPosition();
  private com.revrobotics.CANSparkMax turnMotor1;  // SPARK MAX for Module1
  private final PIDController m_pid1 = new PIDController(1.0, 0.0, 0.0);
  private com.revrobotics.CANSparkMax driveMotor1; // Module1用ドライブモーター

  // ----------------------
  // Module2
  // ----------------------
  double position2 = 0;
  CANcoder encoder2 = new CANcoder(6);
  StatusSignal<Double> positionSignal2 = encoder2.getPosition();
  private com.revrobotics.CANSparkMax turnMotor2;
  private final PIDController m_pid2 = new PIDController(1.0, 0.0, 0.0);
  private com.revrobotics.CANSparkMax driveMotor2; // Module1用ドライブモーター

  // ----------------------
  // Module3
  // ----------------------
  double position3 = 0;
  CANcoder encoder3 = new CANcoder(3);
  StatusSignal<Double> positionSignal3 = encoder3.getPosition();
  private com.revrobotics.CANSparkMax turnMotor3;
  private final PIDController m_pid3 = new PIDController(1.0, 0.0, 0.0);
  private com.revrobotics.CANSparkMax driveMotor3; // Module1用ドライブモーター

  // ----------------------
  // Module4
  // ----------------------
  double position4 = 0;
  CANcoder encoder4 = new CANcoder(12);
  StatusSignal<Double> positionSignal4 = encoder4.getPosition();
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

      encoder1.getConfigurator().apply(config);
    }
    position1 = encoder1.getPosition().getValue();

    turnMotor1 = new com.revrobotics.CANSparkMax(
      8, // ← 仮ID (後で変更可能)
      com.revrobotics.CANSparkMaxLowLevel.MotorType.kBrushless
    );
    turnMotor1.restoreFactoryDefaults();
    driveMotor1 = initializeMotor(7); // ドライブ用

    m_pid1.setTolerance(0.01);
    // m_pid1.enableContinuousInput(0.0, 1.0);


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

      encoder2.getConfigurator().apply(config);
    }
    position2 = encoder2.getPosition().getValue();

    turnMotor2 = new com.revrobotics.CANSparkMax(
      5, // ← 仮ID
      com.revrobotics.CANSparkMaxLowLevel.MotorType.kBrushless
    );
    turnMotor2.restoreFactoryDefaults();
    driveMotor2 = initializeMotor(4); // ドライブ用

    m_pid2.setTolerance(0.01);
    // m_pid2.enableContinuousInput(0.0, 1.0);


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

      encoder3.getConfigurator().apply(config);
    }
    position3 = encoder3.getPosition().getValue();

    turnMotor3 = new com.revrobotics.CANSparkMax(
      2, // 仮ID
      com.revrobotics.CANSparkMaxLowLevel.MotorType.kBrushless
    );
    turnMotor3.restoreFactoryDefaults();
    driveMotor3 = initializeMotor(13); // ドライブ用

    m_pid3.setTolerance(0.01);


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

      encoder4.getConfigurator().apply(config);
    }
    position4 = encoder4.getPosition().getValue();

    turnMotor4 = new com.revrobotics.CANSparkMax(
      11, // 仮ID
      com.revrobotics.CANSparkMaxLowLevel.MotorType.kBrushless
    );
    turnMotor4.restoreFactoryDefaults();
    driveMotor4 = initializeMotor(10); // ドライブ用

    m_pid4.setTolerance(0.01);
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

    // ----------------------------
    // ① Joystick の X軸を取得 + デッドゾーン
    // ----------------------------
    double leftStickX = joystick.getRawAxis(4);
    if (Math.abs(leftStickX) < deadzone) {
      leftStickX = 0.0;
    }
    double driveSpeed = joystick.getRawAxis(1);
    if (Math.abs(driveSpeed) < deadzone) {
      driveSpeed = 0.0;
    }

    // ----------------------------
    // ② -1.0→0.0, +1.0→1.0 にマッピング
    // ----------------------------
    double targetPos = (leftStickX + 1.0) / 2.0;


    // ==================================================
    // Module1: (複製のまま)
    // ==================================================
    position1 = positionSignal1.getValue() - 0.1;
    positionSignal1.refresh();

    double output1 = m_pid1.calculate(position1, targetPos);
    output1 = clamp(output1, -1.0, 1.0);
    turnMotor1.set(output1);
    driveMotor1.set(driveSpeed); // ドライブモーターに速度を適用
    

    SmartDashboard.putNumber("TargetPos1", targetPos);
    SmartDashboard.putNumber("PID Output1", output1);


    // ==================================================
    // Module2
    // ==================================================
    position2 = positionSignal2.getValue() + 0.3;
    positionSignal2.refresh();

    double output2 = m_pid2.calculate(position2, targetPos);
    output2 = clamp(output2, -1.0, 1.0);
    turnMotor2.set(output2);

    SmartDashboard.putNumber("TargetPos2", targetPos);
    SmartDashboard.putNumber("PID Output2", output2);


    // ==================================================
    // Module3
    // ==================================================
    position3 = positionSignal3.getValue() + 0.1;
    positionSignal3.refresh();

    double output3 = m_pid3.calculate(position3, targetPos);
    output3 = clamp(output3, -1.0, 1.0);
    turnMotor3.set(output3);

    SmartDashboard.putNumber("TargetPos3", targetPos);
    SmartDashboard.putNumber("PID Output3", output3);


    // ==================================================
    // Module4
    // ==================================================
    position4 = positionSignal4.getValue();
    positionSignal4.refresh();

    double output4 = m_pid4.calculate(position4, targetPos);
    output4 = clamp(output4, -1.0, 1.0);
    turnMotor4.set(output4);

    SmartDashboard.putNumber("TargetPos4", targetPos);
    SmartDashboard.putNumber("PID Output4", output4);
  }


  // =====================================================
  // clamp メソッド (そのまま複製)
  // =====================================================
  private double clamp(double val, double min, double max) {
    return Math.max(min, Math.min(val, max));
  }


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
