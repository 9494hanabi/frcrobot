package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.controller.PIDController;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;

public class Goal extends SubsystemBase {
    private PWMSparkMax redLineMotor;
    private TalonFX senaTalon;
    private Joystick joystick;
    private PIDController pidController;

    public void setzero(){
        senaTalon.setPosition(0);
    }

    public Goal(Joystick joystick) {
        this.joystick = joystick;

        redLineMotor = new PWMSparkMax(1);
        senaTalon = new TalonFX(17);

        TalonFXConfiguration configsena = new TalonFXConfiguration();
        senaTalon.setNeutralMode(NeutralModeValue.Brake);
        senaTalon.getConfigurator().apply(configsena);
        senaTalon.setPosition(0);
        pidController = new PIDController(0.25,0, 0);
    }

    public double getsenaTalon() {
        return senaTalon.getPosition().getValueAsDouble();
    }

    public void center() {
        double setpoint = 1.25;
        double output = pidController.calculate(getsenaTalon(), setpoint);
        senaTalon.set(output * 0.5);
    }

    public boolean isCentered() {
        double setpoint = 1.25;
        return Math.abs(getsenaTalon() - setpoint) < 0.05; // 目標値との誤差が0.05以下なら「到達」
    }
    
    public void right() {
        double setpoint = 2.25;
        double output = pidController.calculate(getsenaTalon(), setpoint);
        senaTalon.set(output * 0.5);
    }

    public boolean isRight() {
        double setpoint = 2.25;
        return Math.abs(getsenaTalon() - setpoint) < 0.05; // 目標値との誤差が0.05以下なら「到達」
    }

    public void goal(){
        int POVangle = joystick.getPOV();
        double setpoint = 0;
        
        //  今後はPOVのボタンだけでredLineMotorもおけるようにする
        // System.out.println("this is " + POVangle);


        System.out.println("Senatalon encordar :" + senaTalon.getPosition().getValueAsDouble());
        if (POVangle == 270) {
            setpoint = 2.25;
            redLineMotor.set(0.1);
        } else if (POVangle == 90) {
            setpoint = 0.3;
            redLineMotor.set(0.1);
        } else {
            setpoint = 1.25;
            redLineMotor.set(-0.1);

        }

        double output = pidController.calculate(getsenaTalon(), setpoint);
        senaTalon.set(output * 0.5);

        System.out.println("output : " + output);


        // if (POVangle == 90) {
        //     senaTalon.setControl(new PositionVoltage(80.0 / 360.0));
        //     System.out.println("move");
        // } else if (POVangle == 270) {
        //     senaTalon.setControl(new PositionVoltage(-80.0 / 360.0));
        //     System.out.println("gyakusaido");
        // }
    }
}