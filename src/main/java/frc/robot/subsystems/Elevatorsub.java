package frc.robot.subsystems;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.controller.PIDController;


public class Elevatorsub extends SubsystemBase{

    private TalonFX krakenMotorR, krakenMotorL;
    private PIDController pidController;
    
    
        public Elevatorsub() {
            //TODO Auto-generated constructor stub
            krakenMotorR = new TalonFX(15);
            krakenMotorL = new TalonFX(16);
        
            TalonFXConfiguration configR = new TalonFXConfiguration();
            TalonFXConfiguration configL = new TalonFXConfiguration();

            configR.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
            configL.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
            //elev ブレーキモードに
            krakenMotorR.setNeutralMode(NeutralModeValue.Coast);
            krakenMotorL.setNeutralMode(NeutralModeValue.Coast);

            //設定を適用
            krakenMotorR.getConfigurator().apply(configR);
            krakenMotorL.getConfigurator().apply(configL); 
            //elev 初期位置 
            krakenMotorR.setPosition(0);
            krakenMotorL.setPosition(0);

            // PID制御の設定 (P, I, D の値はチューニング必須)
            pidController = new PIDController(0.4, 0.0, 0.0);
            pidController.setTolerance(0.05); // 許容誤差 5cm
        }
    
        public double getElevatorHeightL() {
            return krakenMotorL.getPosition().getValueAsDouble();
        }
    
        public double getElevatorHeightR() {
            return krakenMotorR.getPosition().getValueAsDouble();
        }


        public void setelev(double targetPosition){
            double outputL = pidController.calculate(getElevatorHeightL(), targetPosition);
            double outputR = pidController.calculate(getElevatorHeightR(), targetPosition);
            krakenMotorL.set(-(outputL * 0.3));
            krakenMotorR.set(outputR * 0.3);
        }

        public void downelev(double targetPosition) {
            if (getElevatorHeightR() >= targetPosition) {
                krakenMotorL.set(-0.05);
                krakenMotorR.set(0.05);
            } else {
                return;
            }
            
        }
        

        // public void stopElevator(double targetPosition){
        //     double outputL = pidController.calculate(getElevatorHeightL(),targetPosition);
        //     double outputR = pidController.calculate(getElevatorHeightR(),targetPosition);
        //     krakenMotorL.setControl(new PositionVoltage(outputL));
        //     krakenMotorR.setControl(new PositionVoltage(outputR));
        // }

        public void resetPosition() {
            krakenMotorL.setPosition(0.0);
            krakenMotorR.setPosition(0.0);
        }

}
