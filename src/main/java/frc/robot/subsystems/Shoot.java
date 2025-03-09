package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.SparkLowLevel;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import com.revrobotics.spark.SparkLowLevel;

public class Shoot extends SubsystemBase{
    private PWMSparkMax ShootMotorR;
    private PWMSparkMax ShootMotorL;
    private Joystick joystick;
    private double shootSpeed = 0.5;

    public Shoot(Joystick joystick) {
        this.joystick = joystick;

        ShootMotorR = new PWMSparkMax(9);
    }

    public void ShootBall() {
        boolean keyR = joystick.getRawButton(6);

        if (keyR) {
            ShootMotorR.set(shootSpeed);
        } else {
            ShootMotorR.set(0);
        }
    }
}
