package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import static frc.robot.TriggerUtils.*;
import frc.robot.commands.Climbcom;
import frc.robot.commands.Elevatorcom;
import frc.robot.commands.Goalcom;
import frc.robot.commands.MoveCenter;
import frc.robot.commands.TeleopSwervecom;
import frc.robot.subsystems.*;
import frc.robot.subsystems.Swerve.*;
import java.util.function.DoubleSupplier;

public class RobotContainer {
    
    private final Joystick joystick = new Joystick(0);

    private final DoubleSupplier xSpeedSupplier = () -> MathUtil.applyDeadband(joystick.getRawAxis(1), 0.05);
    private final DoubleSupplier ySpeedSupplier = () -> MathUtil.applyDeadband(joystick.getRawAxis(0), 0.05);
    private final DoubleSupplier rotSpeedSupplier = () -> MathUtil.applyDeadband(-joystick.getRawAxis(4), 0.1);

    private final Elevatorsub elevatorSubsystem = new Elevatorsub();
    private final Swerve swerveSubsystem = new Swerve();
    private final VisionSubsystem visionSubsystem = new VisionSubsystem("photoncamera");
    private final Goal goalSubsystem = new Goal();
    private final Climbsub climbsub = new Climbsub();
    private final ModeManager modeManager = new ModeManager();
    private final SendableChooser<Command> autoChooser = new SendableChooser<>();

    public RobotContainer() {

        autoChooser.setDefaultOption("DefaultAuto", null);
        SmartDashboard.putData("Auto Mode", autoChooser);

        ConfigureDefaultCommands();
        configureButtonBindings();
    }

    private void ConfigureDefaultCommands() {
        elevatorSubsystem.setDefaultCommand(
            new Elevatorcom(
                elevatorSubsystem,
                modeManager,
                () -> joystick.getRawButton(4), // moveUpButton
                () -> joystick.getRawButton(1), // goToL1
                () -> joystick.getRawButton(5), // climbMid
                () -> joystick.getRawButton(7)  // climbDown
            )
        );     
        swerveSubsystem.setDefaultCommand(new TeleopSwervecom(
            swerveSubsystem, 
            xSpeedSupplier, 
            ySpeedSupplier, 
            rotSpeedSupplier,
            joystick));
        goalSubsystem.setDefaultCommand(
            new Goalcom(
                goalSubsystem,
                modeManager,
                () -> joystick.getRawButton(5), // isLeft
                () -> joystick.getRawButton(6), // isRight
                () -> joystick.getRawButton(3) //isShoot
            )
        );
            
        // goalSubsystem.setDefaultCommand(new Goalcom(goalSubsystem));
        // visionSubsystem.setDefaultCommand(new MoveCenter(swerveSubsystem, visionSubsystem));
    }

    private void configureButtonBindings() {
        new JoystickButton(joystick, 3)
            .onTrue(new MoveCenter(swerveSubsystem, visionSubsystem));

        new JoystickButton(joystick, 6) // 巻き取り
            .whileTrue(new Climbcom(climbsub, modeManager, () -> climbsub.pull()));
        
        new JoystickButton(joystick, 8) // 押し出し
            .whileTrue(new Climbcom(climbsub, modeManager, () -> climbsub.push()));
        // POVでモード切り替え

        povEdge(joystick, 0) // POV上
        .onTrue(new InstantCommand(() -> modeManager.moveUp()));
    
        povEdge(joystick, 180) // POV下
        .onTrue(new InstantCommand(() -> modeManager.moveDown()));
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}