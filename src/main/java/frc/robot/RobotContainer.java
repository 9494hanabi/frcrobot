package frc.robot;

import edu.wpi.first.math.MathUtil;
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import static frc.robot.POVEdgeTrigger.*;
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
    private final DoubleSupplier rotSpeedSupplier = () -> MathUtil.applyDeadband(joystick.getRawAxis(4), 0.1);

    private final Elevatorsub elevatorSubsystem = new Elevatorsub();
    private final Swerve swerveSubsystem = new Swerve();
    private final VisionSubsystem visionSubsystem = new VisionSubsystem("photoncamera");
    private final Goal goalSubsystem = new Goal();
    private final Climbsub climbsub = new Climbsub();
    private final ModeManager modeManager = new ModeManager();
    private final SendableChooser<Command> autoChooser = new SendableChooser<>();

    public RobotContainer() {

        // autoChooser.setDefaultOption("New Auto", null);
        // SmartDashboard.putData("Auto Mode", autoChooser);

        ConfigureDefaultCommands();
        configureButtonBindings();
    }

    public Swerve getSwerve() { return swerveSubsystem;}

    private void ConfigureDefaultCommands() {
        elevatorSubsystem.setDefaultCommand(
            new Elevatorcom(
                elevatorSubsystem,
                modeManager,
                () -> joystick.getRawButton(4), // moveUpButton
                () -> joystick.getRawButton(1), // goToL1
                () -> joystick.getRawButton(5) // climbMid
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

        new JoystickButton(joystick, 6) // 巻き取り
            .whileTrue(new Climbcom(climbsub, modeManager, () -> climbsub.pull()));

        new JoystickButton(joystick, 7)
            .onTrue(new InstantCommand(() -> elevatorSubsystem.enableClimbHold()));
        
        new JoystickButton(joystick, 8) // 押し出し
            .whileTrue(new Climbcom(climbsub, modeManager, () -> climbsub.push()));
        new JoystickButton(joystick, 9)
            .onTrue(new InstantCommand(() -> elevatorSubsystem.disableClimbHold()));

        // POVでモード切り替え

        POVEdgeTrigger upTrigger = new POVEdgeTrigger(joystick, 0);
        POVEdgeTrigger downTrigger = new POVEdgeTrigger(joystick, 180);
        
        upTrigger.getTrigger().onTrue(new InstantCommand(() -> modeManager.moveUp()));
        downTrigger.getTrigger().onTrue(new InstantCommand(() -> modeManager.moveDown()));
        
    }

    // public Command getAutonomousCommand() {
    //     return new PathPlannerAuto("New Auto");
    // }
}