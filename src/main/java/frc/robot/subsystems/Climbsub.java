package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.PWMTalonSRX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;;

public class Climbsub extends SubsystemBase{
    private PWMTalonSRX ClimbRight;
    private PWMTalonSRX ClimbLeft;

    public Climbsub () {
    ClimbRight = new PWMTalonSRX(0);
    ClimbLeft = new PWMTalonSRX(2);
    ClimbLeft.setInverted(true);

    }

    public void pull() {
        ClimbLeft.set(-0.5);
        ClimbRight.set(-0.5);
    }

    public void dontpull() {
        ClimbLeft.set(0);
        ClimbRight.set(0);
    }

    public void push() {
        ClimbLeft.set(0.5);
        ClimbRight.set(0.5);
    }
}
