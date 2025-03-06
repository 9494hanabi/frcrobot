package frc.robot.subsystems;

import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;;

public class Climbsub extends SubsystemBase{
    private PWMSparkMax ClimbRight;
    private PWMSparkMax ClimbLeft;

    private Joystick joystick;

    public Climbsub (Joystick joystick) {

    this.joystick = joystick;
    ClimbRight = new PWMSparkMax(9);
    ClimbLeft = new PWMSparkMax(10);

    
    ClimbLeft.setInverted(true);

    }

    public void Climb() {
        boolean keyY = joystick.getRawButton(4);

        if (keyY) {
            ClimbLeft.set(0.5);
            ClimbRight.set(0.5);
        }else{
            ClimbLeft.set(0);
            ClimbRight.set(0);
        }
    }
}
