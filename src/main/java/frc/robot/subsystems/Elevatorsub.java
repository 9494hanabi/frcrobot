package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.math.controller.ElevatorFeedforward;

public class Elevatorsub extends SubsystemBase {
    // エレベーター用モーター（左右独立）
    private final TalonFX leftElevatorMotor;
    private final TalonFX rightElevatorMotor;
    
    // 上昇用 PID とクライミング用 PID（必要に応じて使い分ける）
    private final PIDController pidController;
    private final PIDController climbPidController;
    public Object m_orchestra;
    
    // フィードフォワード（エレベーター特性に合わせたパラメータ）
    // private final ElevatorFeedforward elevatorFF;
    
    public Elevatorsub() {
        // TalonFX の CAN ID をそれぞれ設定
        leftElevatorMotor = new TalonFX(16);
        rightElevatorMotor = new TalonFX(15);
        
        // モーター設定用コンフィグ
        TalonFXConfiguration configLeft = new TalonFXConfiguration();
        TalonFXConfiguration configRight = new TalonFXConfiguration();
        
        // ブレーキモード設定
        leftElevatorMotor.setNeutralMode(NeutralModeValue.Brake);
        rightElevatorMotor.setNeutralMode(NeutralModeValue.Brake);
        
        // 設定を適用
        leftElevatorMotor.getConfigurator().apply(configLeft);
        rightElevatorMotor.getConfigurator().apply(configRight);
        
        // 初期位置リセット
        leftElevatorMotor.setPosition(0);
        rightElevatorMotor.setPosition(0);
        
        // PID コントローラの初期化
        pidController = new PIDController(0.4, 0.0, 0.0);
        pidController.setTolerance(0.005);
        
        climbPidController = new PIDController(0.4, 0.0, 0.0);
        climbPidController.setTolerance(0.005);
    }
    
    // 現在の左・右エレベーターの位置（高さ）を取得
    public double getElevatorHeightLeft() {
        return leftElevatorMotor.getPosition().getValueAsDouble();
    }
    
    public double getElevatorHeightRight() {
        return rightElevatorMotor.getPosition().getValueAsDouble();
    }
    
    /**
     * 目標位置へ PID 制御で移動させる。
     * 左右は逆方向の目標値で制御する。
     */
    public void setElevatorPosition(double targetPosition) {
        double outputLeft = pidController.calculate(getElevatorHeightLeft(), targetPosition);
        double outputRight = pidController.calculate(getElevatorHeightRight(), -targetPosition);
        System.out.println("left: " + outputLeft);
        System.out.println("right: " + outputRight);
        leftElevatorMotor.set(outputLeft * 0.05);
        rightElevatorMotor.set(outputRight * 0.05);
    }
    
    /**
     * 指定の位置より上にある場合、ゆっくり下降させる
     */
    public void moveDown(double targetPosition) {
        double outputLeft = pidController.calculate(getElevatorHeightLeft(), targetPosition);
        double outputRight = pidController.calculate(getElevatorHeightRight(), -targetPosition);
        if (getElevatorHeightLeft() >= targetPosition) {
            leftElevatorMotor.set(0.02 + (outputLeft * 0.03));
            rightElevatorMotor.set(-0.02 + (outputRight * 0.03));
        }
    }

    public void climbDown(double targetPosition) {
        double outputLeft = pidController.calculate(getElevatorHeightLeft(), targetPosition);
        double outputRight = pidController.calculate(getElevatorHeightRight(), -targetPosition);
        if (getElevatorHeightLeft() >= targetPosition) {
            leftElevatorMotor.set(0.02 + (outputLeft * 0.18));
            rightElevatorMotor.set(-0.02 + (outputRight * 0.18));
        }
    }
    
    // エレベーターの位置をリセット
    public void resetPosition() {
        leftElevatorMotor.setPosition(0.0);
        rightElevatorMotor.setPosition(0.0);
    }

    public boolean isAtPosition(double targetPosition) {
        double tolerance = 0.5;
        
        boolean leftAtPosition = Math.abs(getElevatorHeightLeft() - targetPosition) < tolerance;
        boolean rightAtPosition = Math.abs(getElevatorHeightRight() + targetPosition) < tolerance; // 右は負の方向なので符号反転
    
        return leftAtPosition && rightAtPosition;
    }

    public void stopElevator() {
        leftElevatorMotor.set(0.02);
        rightElevatorMotor.set(0.02);
    }
}
