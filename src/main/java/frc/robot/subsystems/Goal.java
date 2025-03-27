package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;

public class Goal extends SubsystemBase {
    private PWMSparkMax redLineMotor;
    private TalonFX senaTalon;
    private PIDController pidController;

    public void setzero(){
        senaTalon.setPosition(0);
    }

    public Goal() {

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

    public void goal(boolean isLeft, boolean isRight, boolean isShoot) {
        double setpoint;
    
        if (isLeft) {
            setpoint = -1.95;
            if (isShoot) {
                redLineMotor.set(0.5);
            }
        } else if (isRight) {
            setpoint = -0.5;
            if (isShoot) {
                redLineMotor.set(0.5);
            }
        } else {
            setpoint = -1.33;
            redLineMotor.set(-0.1); // 常に軽く回す（吸引なし）
        }
    
        double output = pidController.calculate(getsenaTalon(), setpoint);
        senaTalon.set(output * 0.5);
    }
    

}