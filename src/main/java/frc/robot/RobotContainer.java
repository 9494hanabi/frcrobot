package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.Elevatorcom;
import frc.robot.subsystems.Elevatorsub;

public class RobotContainer {
    private final Elevatorsub elevatorSubsystem = new Elevatorsub();
    private final Joystick joystick = new Joystick(0);

    public RobotContainer() {
        configureBindings();

        // デフォルトコマンドを設定（常に実行される）
        elevatorSubsystem.setDefaultCommand(new Elevatorcom(elevatorSubsystem, joystick, 1, 2));
    }

    private void configureBindings() {
        // Aボタン（1）で上昇、Bボタン（2）で下降
        new JoystickButton(joystick, 1).onTrue(new Elevatorcom(elevatorSubsystem, joystick, 1, 2));
    }
}