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
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;


public class Robot extends TimedRobot {

  // =====================================================
  // もともと "1つのモジュール" 用に書かれていた変数を
  // 4セット分複製して用意 (Module1〜Module4)
  // =====================================================

  // ----------------------
  // 共通: Joystick
  // ----------------------
  Joystick joystick = new Joystick(0);
  private AHRS navx;



  // ----------------------
  // Module1
  // ----------------------
  double position1 = 0;                      // (0.0~1.0)
  CANcoder encoder1 = new CANcoder(9);       // CANCoder(9)
  StatusSignal<Double> positionSignal1 = encoder1.getAbsolutePosition();
  private com.revrobotics.CANSparkMax turnMotor1;  // SPARK MAX for Module1
  private final PIDController m_pid1 = new PIDController(1, 0.001, 0.01);
  private com.revrobotics.CANSparkMax driveMotor1; // Module1用ドライブモーター

  // ----------------------
  // Module2
  // ----------------------
  double position2 = 0;
  CANcoder encoder2 = new CANcoder(6);
  StatusSignal<Double> positionSignal2 = encoder2.getAbsolutePosition();
  private com.revrobotics.CANSparkMax turnMotor2;
  private final PIDController m_pid2 = new PIDController(1, 0.001, 0.01);
  private com.revrobotics.CANSparkMax driveMotor2; // Module2用ドライブモーター

  // ----------------------
  // Module3
  // ----------------------
  double position3 = 0;
  CANcoder encoder3 = new CANcoder(3);
  StatusSignal<Double> positionSignal3 = encoder3.getAbsolutePosition();
  private com.revrobotics.CANSparkMax turnMotor3;
  private final PIDController m_pid3 = new PIDController(1, 0.001, 0.01);
  private com.revrobotics.CANSparkMax driveMotor3; // Module3用ドライブモーター

  // ----------------------
  // Module4
  // ----------------------
  double position4 = 0;
  CANcoder encoder4 = new CANcoder(12);
  StatusSignal<Double> positionSignal4 = encoder4.getAbsolutePosition();
  private com.revrobotics.CANSparkMax turnMotor4;
  private final PIDController m_pid4 = new PIDController(1, 0.001, 0.01);
  private com.revrobotics.CANSparkMax driveMotor4; // Module4用ドライブモーター
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

    //====navx====
    navx = new AHRS(SPI.Port.kMXP);

    // ==================================================
    // Module1: 複製元コードと同じ処理
    // ==================================================32
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

    //teleopじゃなくてこっちで読む
    positionSignal1.refresh();
    positionSignal2.refresh();
    positionSignal3.refresh();
    positionSignal4.refresh();


    position1 = positionSignal1.getValue() -0.07;
    position2 = positionSignal2.getValue() -0.697;
    position3 = positionSignal3.getValue() -0.432;
    position4 = positionSignal4.getValue() -0.99;

  }


  // =====================================================
  // teleopPeriodic
  //   (複製元コードと同じロジックをモジュールごとに)
  // =====================================================
  @Override
  public void teleopPeriodic() {

    //===navx===
    double yaw_deg = navx.getYaw();

    double yaw_rad = Math.toRadians(yaw_deg); // ラジアンに変換
    double cosA = Math.cos(yaw_rad);
    double sinA = Math.sin(yaw_rad);



    //robot size
    double  RobotWith = 6.75;
    double  RobotLength = 6.75;
    double  R =  Math.sqrt( RobotLength*RobotLength + RobotWith * RobotWith);


    //offset
    double lastangle1 = 0.0;
    double lastangle2 = 0.0;
    double lastangle3 = 0.0;
    double lastangle4 = 0.0;

    // ----------------------------
    // ① Joystick の X軸を取得 + デッドゾーン
    // ----------------------------
    double X = -joystick.getRawAxis(0);
    double Y = -joystick.getRawAxis(1);
    if (Math.abs(X) < deadzone) {
      X = 0;
    }
    if (Math.abs(Y) < deadzone) {
        // driveMotor1.set(0);
        // driveMotor2.set(0);
        // driveMotor3.set(0);
        // driveMotor4.set(0);
        Y = 0;
      }

    double Rotate = -joystick.getRawAxis(4);
    if (Math.abs(Rotate) < deadzone) {
      Rotate = 0.0;
    
    }

    double fieldX =  X * cosA - Y * sinA;
    double fieldY =  X * sinA + Y * cosA;

    //べくとるせってい
    double A = fieldX - Rotate * (RobotWith / R);
    double B = fieldX + Rotate * (RobotWith / R);
    double C = fieldY - Rotate * (RobotLength / R);
    double D = fieldY + Rotate * (RobotLength / R);

    //module1
    double FRspeed = Math.sqrt(A*A + C*C);
    double FRrad = Math.toDegrees(Math.atan2(A,C));
    //module2
    double BLspeed = Math.sqrt(B*B+ D*D);
    double BLrad = Math.toDegrees(Math.atan2(B,D));
    //module3
    double BRspeed = Math.sqrt(B*B + C*C);
    double BRrad = Math.toDegrees(Math.atan2(B,C));
    //module4
    double FLspeed = Math.sqrt(A*A + D*D);
    double FLrad = Math.toDegrees(Math.atan2(A,D));

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
    double desiredAngleDeg1 = FRrad;
    double desiredAngleDeg2 = BLrad;
    double desiredAngleDeg3 = BRrad;
    double desiredAngleDeg4 = FLrad;

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
    //おわり

        
    // ==================================================
    // Module1: (複製のまま)
    // ==================================================
    
    
    double actualAngleDeg1 = position1 * 360.0;
    double delta1 = desiredAngleDeg1 - actualAngleDeg1;
    // -180～+180 の範囲に正規化

    
    while (delta1 > 180)  delta1 -= 360;
    while (delta1 < -180) delta1 += 360;

    // もし誤差の絶対値が90°を超えるなら、
    // 角度を180°ズラして speed の符号を反転
    if (Math.abs(delta1) > 90) {
        if (delta1 > 0) {
            desiredAngleDeg1 -= 180;
        } else {
            desiredAngleDeg1 += 180;
        }
        FRspeed = -FRspeed;
    }

    double targetPos1 = (desiredAngleDeg1 % 360.0) / 360.0;
    

    

    double output1 = m_pid1.calculate(position1, targetPos1);
    //output1 = clamp(output1, -1.0, 1.0);
    //lastangle1 = output1;

    if (Math.abs(FRspeed) < 0.05) {
      output1 = lastangle1;
      } else {
          lastangle1 = output1; // 現在の目標角度を更新
      }
    turnMotor1.set(lastangle1);
    driveMotor1.set(FRspeed*0.5); // ドライブモーターに速度を適
 


    // ==================================================
    // Module2
    // ==================================================
    
    
    double actualAngleDeg2 = position2 * 360.0;
    double delta2 = desiredAngleDeg2 - actualAngleDeg2;

    
    // -180～+180 の範囲に正規化
    while (delta2 > 180)  delta2 -= 360;
    while (delta2 < -180) delta2 += 360;

    // もし誤差の絶対値が90°を超えるなら、
    // 角度を180°ズラして speed の符号を反転
    if (Math.abs(delta2) > 90) {
        if (delta2 > 0) {
            desiredAngleDeg2 -= 180;
        } else {
            desiredAngleDeg2 += 180;
        }
        BLspeed = -BLspeed;
    }

    double targetPos2 = (desiredAngleDeg2 % 360.0) / 360.0;
    

    

    double output2 = m_pid2.calculate(position2, targetPos2);
    //output2 = clamp(output2, -1.0, 1.0);
        if (Math.abs(BLspeed) < 0.05) {
      output2 = lastangle2;
      } else {
          lastangle2 = output2; // 現在の目標角度を更新
      }
    turnMotor2.set(lastangle2);
    driveMotor2.set(-BLspeed*0.5); 


    // ==================================================
    // Module3
    // ==================================================
    
    
    double actualAngleDeg3 = position3 * 360.0;
    double delta3 = desiredAngleDeg3 - actualAngleDeg3;
    
    
    // -180～+180 の範囲に正規化
    while (delta3 > 180)  delta3 -= 360;
    while (delta3 < -180) delta3 += 360;

    // もし誤差の絶対値が90°を超えるなら、
    // 角度を180°ズラして speed の符号を反転
    if (Math.abs(delta3) > 90) {
        if (delta3 > 0) {
            desiredAngleDeg3 -= 180;
        } else {
            desiredAngleDeg3 += 180;
        }
        BRspeed = -BRspeed;
    }

    double targetPos3 = (desiredAngleDeg3 % 360.0) / 360.0;
    
   

    double output3 = m_pid3.calculate(position3, targetPos3);
    //output3 = clamp(output3, -1.0, 1.0);
      if (Math.abs(BRspeed) < 0.05) {
      output3 = lastangle3;
      } else {
          lastangle3 = output3; // 現在の目標角度を更新
      }
    turnMotor3.set(lastangle3);
    driveMotor3.set(BRspeed*0.5); 


    // ==================================================
    // Module4
    // ==================================================
    
    
    double actualAngleDeg4 = position4 * 360.0;
    double delta4 = desiredAngleDeg4 - actualAngleDeg4;

    // -180～+180 の範囲に正規化
    while (delta4 > 180)  delta4 -= 360;
    while (delta4 < -180) delta4 += 360;

    // もし誤差の絶対値が90°を超えるなら、
    // 角度を180°ズラして speed の符号を反転
    if (Math.abs(delta4) > 90) {
        if (delta4 > 0) {
            desiredAngleDeg4 -= 180;
        } else {
            desiredAngleDeg4 += 180;
        }
        FLspeed = -FLspeed;
    }
    
    double targetPos4 = (desiredAngleDeg4 % 360.0) / 360.0;

    double output4 = m_pid4.calculate(position4, targetPos4);
    //output4 = clamp(output4, -1.0, 1.0);
      if (Math.abs(FLspeed) < 0.05) {
      output4 = lastangle4;
      } else {
          lastangle4 = output4; // 現在の目標角度を更新
      }
    turnMotor4.set(lastangle4);
    driveMotor4.set(FLspeed*0.5); 

    SmartDashboard.putNumber("Actual Angle1", actualAngleDeg1);
    SmartDashboard.putNumber("Desired Angle1", desiredAngleDeg1);
    SmartDashboard.putNumber("Angle Error1", delta1);

    SmartDashboard.putNumber("Actual Angle2", actualAngleDeg2);
    SmartDashboard.putNumber("Desired Angle2", desiredAngleDeg2);
    SmartDashboard.putNumber("Angle Error2", delta2);

    SmartDashboard.putNumber("Actual Angle3", actualAngleDeg3);
    SmartDashboard.putNumber("Desired Angle3", desiredAngleDeg3);
    SmartDashboard.putNumber("Angle Error3", delta3);

    SmartDashboard.putNumber("Actual Angle4", actualAngleDeg4);
    SmartDashboard.putNumber("Desired Angle4", desiredAngleDeg4);
    SmartDashboard.putNumber("Angle Error4", delta4);
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


// わっしょいはなび 関数作りたい
