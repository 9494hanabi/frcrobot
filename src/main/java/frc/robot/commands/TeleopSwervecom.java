package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Swerve.Swerve;
import java.util.function.DoubleSupplier;


public class TeleopSwervecom extends Command{
    private final Swerve swerveSubsystem;
    private final DoubleSupplier forwardSupplier;
    private final DoubleSupplier starfeSupplier;
    private final DoubleSupplier turnSupplier;
    private boolean resetButtonPreviouslyPressed = false;

    private Joystick joystick;

    public TeleopSwervecom(Swerve swerveSubsystem, 
                            DoubleSupplier forwardSupplier, 
                            DoubleSupplier starfeSupplier, 
                            DoubleSupplier turnSupplier,
                            Joystick joystick) {
        this.swerveSubsystem = swerveSubsystem;
        this.forwardSupplier = forwardSupplier;
        this.starfeSupplier = starfeSupplier;
        this.turnSupplier = turnSupplier;
        this.joystick = joystick;
        addRequirements(swerveSubsystem);
    }

    @Override
    public void execute() {
        
        double forwardInput = forwardSupplier.getAsDouble();
        double starfeInput = starfeSupplier.getAsDouble();
        double turnInput = turnSupplier.getAsDouble();

        double xSpeed = -forwardInput * swerveSubsystem.getMaxLinearVelocityMetersPerSec();
        double ySpeed = -starfeInput * swerveSubsystem.getMaxLinearVelocityMetersPerSec();
        double rotationSpeed = -turnInput * swerveSubsystem.getMaxAngularVelocityRadiansPerSec();

        

        if (Math.abs(xSpeed) < 0.01) xSpeed = 0;
        if (Math.abs(ySpeed) < 0.01) ySpeed = 0;
        if (Math.abs(rotationSpeed) < 0.01) rotationSpeed = 0;

        Rotation2d robotRotaion = swerveSubsystem.getHeading().unaryMinus();
        ChassisSpeeds fieldRlativeSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rotationSpeed, robotRotaion);

        boolean resetButtonPressed = joystick.getRawButton(3);

        if (resetButtonPressed && !resetButtonPreviouslyPressed) {
            swerveSubsystem.resetHeading();
        }

        swerveSubsystem.drive(fieldRlativeSpeeds, null);

    }

    @Override
    public void end(boolean interrupted) {
        swerveSubsystem.drive(new ChassisSpeeds(0, 0, 0), null);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
