package frc.robot.subsystems;


import java.io.File;

import com.ctre.phoenix6.hardware.CANcoder;

/**
 * Initialize {@link SwerveDrive} with the directory provided.
 *
 * @param directory Directory of swerve drive config files.
 */
public class SwerveSubsystem{

public SwerveSubsystem(File directory) {
    // Angle conversion factor is 360 / (GEAR RATIO * ENCODER RESOLUTION)
    // In this case the gear ratio is 12.8 motor revolutions per wheel rotation.
    // The encoder resolution per motor revolution is 1 per motor revolution.
    double angleConversionFactor = SwerveMath.calculateDegreesPerSteeringRotation(12.8, 1);

    // Motor conversion factor is (PI * WHEEL DIAMETER IN METERS) / (GEAR RATIO * ENCODER RESOLUTION).
    // In this case the wheel diameter is 4 inches, which must be converted to meters to get meters/second.
    // The gear ratio is 6.75 motor revolutions per wheel rotation.
    // The encoder resolution per motor revolution is 1 per motor revolution.
    double driveConversionFactor = SwerveMath.calculateMetersPerRotation(Units.inchesToMeters(4), 6.75, 1);

    // Configure the Telemetry before creating the SwerveDrive to avoid unnecessary objects being created.
    SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;

    try {
        swerveDrive = new SwerveParser(directory).createSwerveDrive(maximumSpeed, angleConversionFactor, driveConversionFactor);
    } catch (Exception e) {
        throw new RuntimeException(e);
    }

    swerveDrive.setHeadingCorrection(false); // Heading correction should only be used while controlling the robot via angle.

    for (SwerveModule m : swerveDrive.getModules()) {
        System.out.println("Module Name: " + m.configuration.name);
        CANcoder absoluteEncoder = (CANcoder) m.configuration.absoluteEncoder.getAbsoluteEncoder();
    }
}
}


