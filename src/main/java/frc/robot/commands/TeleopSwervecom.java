package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Swerve.Swerve;
import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj.Timer;



public class TeleopSwervecom extends Command{
    private final Swerve swerveSubsystem;
    private final DoubleSupplier forwardSupplier;
    private final DoubleSupplier starfeSupplier;
    private final DoubleSupplier turnSupplier;

    private final Timer autoTimer = new Timer();
    private boolean isAutoStarted = false;



    public TeleopSwervecom(Swerve swerveSubsystem, 
                            DoubleSupplier forwardSupplier, 
                            DoubleSupplier starfeSupplier, 
                            DoubleSupplier turnSupplier,
                            Joystick joystick) {
        this.swerveSubsystem = swerveSubsystem;
        this.forwardSupplier = forwardSupplier;
        this.starfeSupplier = starfeSupplier;
        this.turnSupplier = turnSupplier;
        addRequirements(swerveSubsystem);
    }

    @Override
    public void initialize() {
    if (DriverStation.isAutonomousEnabled()) {
        autoTimer.reset();
        autoTimer.start();
        isAutoStarted = true;
    }
}


    @Override
    public void execute() {

        if (DriverStation.isAutonomousEnabled() && autoTimer.get() > 1.0) {
            swerveSubsystem.drive(new ChassisSpeeds(0, 0, 0), null);
            return;
        }

        double forwardInput = forwardSupplier.getAsDouble()*0.5;
        double starfeInput = starfeSupplier.getAsDouble()*0.5;
        double turnInput = -turnSupplier.getAsDouble();

        double xSpeed = -forwardInput * swerveSubsystem.getMaxLinearVelocityMetersPerSec();
        double ySpeed = -starfeInput * swerveSubsystem.getMaxLinearVelocityMetersPerSec();
        double rotationSpeed = -turnInput * swerveSubsystem.getMaxAngularVelocityRadiansPerSec();

        

        if (Math.abs(xSpeed) < 0.03) xSpeed = 0;
        if (Math.abs(ySpeed) < 0.03) ySpeed = 0;
        if (Math.abs(rotationSpeed) < 0.03) rotationSpeed = 0;

        Rotation2d robotRotaion = swerveSubsystem.getHeading().unaryMinus();
        ChassisSpeeds fieldRlativeSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rotationSpeed, robotRotaion);

        if (DriverStation.isAutonomousEnabled()) {
            swerveSubsystem.drive(new ChassisSpeeds(-0.2, 0, 0), null);
        } else {
                swerveSubsystem.drive(fieldRlativeSpeeds, null);
        }
        

    }

    @Override
    public void end(boolean interrupted) {
        swerveSubsystem.drive(new ChassisSpeeds(0, 0, 0), null);
    }

    @Override
    public boolean isFinished() {
        return DriverStation.isAutonomousEnabled() && autoTimer.hasElapsed(1.0);
    }
    
}
