package frc.robot;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj.Joystick;

public class TriggerUtils {
    private static int lastPOV = -1;
    
    /**
     * POVが指定の角度のときにtrueになるトリガーを返す。
     */
    public static Trigger povAngle(Joystick joystick, int angle) {
        return new Trigger(() -> joystick.getPOV() == angle);
    }

    /**
     * 指定のボタンが押されたときにtrueになるトリガーを返す。
     */
    public static Trigger buttonPressed(Joystick joystick, int button) {
        return new Trigger(() -> joystick.getRawButton(button));
    }

    public static Trigger povEdge(Joystick joystick, int targetAngle) {
        return new Trigger(() -> {
            int current = joystick.getPOV();
            boolean isEdge = current == targetAngle && lastPOV != targetAngle;
            lastPOV = current;
            return isEdge;
        });
    }
}
