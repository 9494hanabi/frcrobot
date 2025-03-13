package frc.robot.subsystems;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.hardware.TalonFX;
// import com.ctre.phoenix6.motorcontrol.ControlMode;

import java.util.ResourceBundle.Control;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.signals.ControlModeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ElevatorFeedforward;

public class Elevatorsub extends SubsystemBase{

    private TalonFX krakenMotorR, krakenMotorL;
    private PIDController pidController;
    private PIDController climbPidController;
    private ElevatorFeedforward elevatorFF;
    
    
        public Elevatorsub() {
            //TODO Auto-generated constructor stub
            krakenMotorR = new TalonFX(15);
            krakenMotorL = new TalonFX(16);
        
            TalonFXConfiguration configR = new TalonFXConfiguration();
            TalonFXConfiguration configL = new TalonFXConfiguration();

            elevatorFF = new ElevatorFeedforward(0.2, 0.5, 2.0, 0.1);

            configR.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
            configL.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
            //elev ブレーキモードに
            krakenMotorR.setNeutralMode(NeutralModeValue.Brake);
            krakenMotorL.setNeutralMode(NeutralModeValue.Brake);

            //設定を適用
            krakenMotorR.getConfigurator().apply(configR);
            krakenMotorL.getConfigurator().apply(configL); 
            //elev 初期位置 
            krakenMotorR.setPosition(0);
            krakenMotorL.setPosition(0);

            // PID制御の設定 (P, I, D の値はチューニング必須)
            pidController = new PIDController(0.4, 0.0, 0.0);
            pidController.setTolerance(0.05); // 許容誤差 5cm

            climbPidController = new PIDController(0.4, 0.0, 0.0);
            climbPidController.setTolerance(0.05); // 許容誤差 5cm

            
        }
    
        public double getElevatorHeightL() {
            return krakenMotorL.getPosition().getValueAsDouble();
        }
    
        public double getElevatorHeightR() {
            return krakenMotorR.getPosition().getValueAsDouble();
        }


        public void setelev(double targetPosition){
            double outputL = pidController.calculate(getElevatorHeightL(), -(targetPosition));
            double outputR = pidController.calculate(getElevatorHeightR(), targetPosition);
            System.out.println("outputL: " + outputL);
            System.out.println("outputR: " + outputR);
            System.out.println("heightL: " + getElevatorHeightL());
            System.out.println("heightR: " + getElevatorHeightR());
            krakenMotorL.set(outputL * 0.4);
            krakenMotorR.set(outputR * 0.4);
        }

        public void downelev(double targetPosition) {
            if (getElevatorHeightR() >= targetPosition) {
                krakenMotorL.set(-0.05);
                krakenMotorR.set(0.05);
            } else {
                return;
            }
        }

        public void climbElevator() {
            double velocity = krakenMotorR.getVelocity().getValueAsDouble();
            double acceleration = krakenMotorR.getAcceleration().getValueAsDouble();
            double voltage = elevatorFF.calculate(velocity, acceleration);

            // System.out.println("voltage now: " + voltage);
            // System.out.println("voltage as motors now: " + voltage / 12);

            double outputL = climbPidController.calculate(getElevatorHeightL(), 0.18);
            double outputR = climbPidController.calculate(getElevatorHeightR(), 0.18);
            if (getElevatorHeightR() > 0.1) {
                krakenMotorL.set(0.87);
                krakenMotorR.set(-0.87);
            }
            System.out.println("outputLR now: " + outputR);
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
