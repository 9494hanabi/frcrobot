package frc.robot.subsystems;

import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.motorcontrol.PWMTalonFX;
import edu.wpi.first.wpilibj.motorcontrol.PWMTalonSRX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;;

public class Climbsub extends SubsystemBase{
    private PWMTalonSRX ClimbRight;
    private PWMTalonSRX ClimbLeft;

    private Joystick joystick;


    public Climbsub (Joystick joystick) {

    this.joystick = joystick;
    ClimbRight = new PWMTalonSRX(0);
    ClimbLeft = new PWMTalonSRX(2);

    
    ClimbLeft.setInverted(true);

    }

    public void Climb() {
        boolean climbKey = joystick.getRawButton(8);

        if (climbKey) {
            ClimbLeft.set(-0.5);
            ClimbRight.set(-0.5);
            System.out.println("button pushed");
        }else{
            ClimbLeft.set(0);
            ClimbRight.set(0);
        }
    }
}
