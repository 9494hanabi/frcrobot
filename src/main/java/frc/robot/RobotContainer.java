package frc.robot;

import choreo.auto.AutoChooser;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.Elevatorcom;
import frc.robot.subsystems.Elevatorsub;
import com.pathplanner.lib.auto.AutoBuilder;

public class RobotContainer {
    private final Elevatorsub elevatorSubsystem = new Elevatorsub();
    private final Joystick joystick = new Joystick(0);
    private final SendableChooser<Command> autoChooser;
    public Command getAutonomusCommand;

    public RobotContainer() {
        configureBindings();

        // デフォルトコマンドを設定（常に実行される）
        elevatorSubsystem.setDefaultCommand(new Elevatorcom(elevatorSubsystem, joystick, 1, 2));

        autoChooser = AutoBuilder.buildAutoChooser("New Auto");
        SmartDashboard.putData("Auto mode", autoChooser);

        configureBindings();
    }

    private void configureBindings() {
        // Aボタン（1）で上昇、Bボタン（2）で下降
        new JoystickButton(joystick, 1).onTrue(new Elevatorcom(elevatorSubsystem, joystick, 1, 2));
    }

    Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }


    
}