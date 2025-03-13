package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
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

        redLineMotor = new PWMSparkMax(7);
        senaTalon = new TalonFX(17);

        TalonFXConfiguration configsena = new TalonFXConfiguration();
        senaTalon.setNeutralMode(NeutralModeValue.Brake);
        senaTalon.getConfigurator().apply(configsena);
        senaTalon.setPosition(0);
        pidController = new PIDController(0.4, 0.0, 0.0);

    }

    public double getsenaTalon() {
        return senaTalon.getPosition().getValueAsDouble();
    }

    public void goal(){
        boolean keyX = joystick.getRawButton(5);
        boolean keyL = joystick.getRawButton(6);
        int POVangle = joystick.getPOV();
        double setpoint = 0;
        
        //  今後はPOVのボタンだけでredLineMotorもおけるようにする
        // System.out.println("this is " + POVangle);

        if (keyX) {
            redLineMotor.set(0.1);
        }else if (keyL) {
            redLineMotor.set(-0.5);
        }else{
            redLineMotor.set(0);
        }

        // System.out.println("senaTalon is " + getsenaTalon());

        if (POVangle == 270){
            double output = pidController.calculate(getsenaTalon(), 0.2);
            senaTalon.set(output);
        }

        // if (POVangle == 90) {
        //     setpoint = 0.5;
        //     redLineMotor.set(-0.2);
        // } else if (POVangle == 270) {
        //     setpoint = 0.0;
        //     redLineMotor.set(-0.2);
        // } else {
        //     setpoint = 0.25;
        //     redLineMotor.set(0.2);
        // }

        // double output = pidController.calculate(getsenaTalon(), setpoint);
        // senaTalon.set(output);


        // if (POVangle == 90) {
        //     senaTalon.setControl(new PositionVoltage(80.0 / 360.0));
        //     System.out.println("move");
        // } else if (POVangle == 270) {
        //     senaTalon.setControl(new PositionVoltage(-80.0 / 360.0));
        //     System.out.println("gyakusaido");
        // }
    }
}