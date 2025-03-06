package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;

public class Goal extends SubsystemBase {
    private PWMSparkMax goalMax;
    private TalonFX senaTalon;
    private Joystick joystick;

    public Goal(Joystick joystick) {
        this.joystick = joystick;

        goalMax = new PWMSparkMax(7);
        senaTalon = new TalonFX(18);

        TalonFXConfiguration configsena = new TalonFXConfiguration();
        senaTalon.setNeutralMode(NeutralModeValue.Brake);
        senaTalon.getConfigurator().apply(configsena);
        senaTalon.setPosition(0);

    }

    public void goal(){
        boolean keyX = joystick.getRawButton(3);
        boolean keyL = joystick.getRawButton(5);
        int POVangle = joystick.getPOV();
        double setRight = 15;
        double setLeft = -15;

        if (keyX) {
            goalMax.set(0.1);
        }else if (keyL) {
            goalMax.set(-0.5);
        }else{
            goalMax.set(0);
        }
        if (POVangle == 90) {
            senaTalon.setControl(new PositionVoltage(90 / 360));
        } else if (POVangle == -90) {
            senaTalon.setControl(new PositionVoltage(-90 / 360));
        }
    }
}
