package frc.robot.subsystems.Swerve;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.util.DriveFeedforwards;
import com.studica.frc.AHRS;

public class Swerve extends SubsystemBase {
  private Module[] modules;
  private SwerveDriveOdometry odometry;
  public SwerveDriveKinematics kinematics;
  public AHRS navx = new AHRS(AHRS.NavXComType.kMXP_SPI);

  private final double trackWidhMeters = 0.675;
  private final double trackLengthMeters = 0.675;

  // ロボットの中心からNavXまでのオフセット（前方に0.2m）
  private final Translation2d navxOffset = new Translation2d(0.0, -0.25); 

  private final double limitspeed = 1;
  private final double maxLinearVelocityMetersPerSec = Module.getwheelMaxLinearVelocity() / limitspeed;
  private final double maxAngularVelocityRadiansPerSec = (Module.getwheelMaxLinearVelocity() / limitspeed) / Math.hypot(trackLengthMeters, trackWidhMeters);

  public Swerve() {

    modules = new Module[] {new Module(0), new Module(1), new Module(2), new Module(3)};

    kinematics = new SwerveDriveKinematics(
      new Translation2d(trackWidhMeters / 2, trackWidhMeters / 2),
      new Translation2d(trackWidhMeters / 2, -trackWidhMeters / 2),
      new Translation2d(-trackWidhMeters / 2, trackWidhMeters / 2),
      new Translation2d(-trackWidhMeters / 2, -trackWidhMeters / 2)
    );

    SwerveModulePosition[] modulePositions = new SwerveModulePosition[] {
      new SwerveModulePosition(modules[0].getDistanceMeters(), modules[0].getAngle()),
      new SwerveModulePosition(modules[1].getDistanceMeters(), modules[1].getAngle()),
      new SwerveModulePosition(modules[2].getDistanceMeters(), modules[2].getAngle()),
      new SwerveModulePosition(modules[3].getDistanceMeters(), modules[3].getAngle())
    };

    odometry = new SwerveDriveOdometry(kinematics, navx.getRotation2d(), modulePositions, new Pose2d(0,0, new Rotation2d()));
  }

  private SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] positions = new SwerveModulePosition[modules.length];
    for (int i = 0; i < modules.length; i++) {
      positions[i] = modules[i].getPosition();
    }
    return positions;
  }

  public Rotation2d getHeading() {
    return Rotation2d .fromDegrees(navx.getYaw() - 21);
  }

  public double getYaw() {
    return navx.getYaw();
  }

  public boolean isConnected() {
    return navx.isConnected();
  }

  //soutai seigyo
  public void drive(ChassisSpeeds desiredSpeeds, DriveFeedforwards feedforwards) {
    SwerveModuleState[] targetModuleSpeed = kinematics.toSwerveModuleStates(desiredSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(targetModuleSpeed, Module.getwheelMaxLinearVelocity());

    for (int i = 0; i < modules.length; i++) {
      modules[i].run(targetModuleSpeed[i]);
    }

    odometry.update(navx.getRotation2d(), new SwerveModulePosition[]{
      modules[0].getPosition(),
      modules[1].getPosition(),
      modules[2].getPosition(),
      modules[3].getPosition(),
    });
  }

  //not soutai seigyo
  public void notrelativedrive(ChassisSpeeds desiredSpeeds){
    SwerveModuleState[] targetModuleSpeed = kinematics.toSwerveModuleStates(desiredSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(targetModuleSpeed, Module.getwheelMaxLinearVelocity());

    for (int i = 0; i < modules.length; i++) {
      modules[i].run(targetModuleSpeed[i]);
    }
  }

  public Pose2d getPose() {
        // 現在のポーズを取得
        Pose2d currentPose = odometry.getPoseMeters();
        
        // NavXのオフセットを適用
        return currentPose.transformBy(new Transform2d(navxOffset, new Rotation2d()));
    }

  public void resetOdometry(Pose2d pose) {
    odometry.resetPosition(navx.getRotation2d(), getModulePositions(), pose);
  }

  public void resetHeading() {
    navx.reset();
    odometry.resetPosition(
        Rotation2d.fromDegrees(0), // navx resetした直後 = 0度を前提
        getModulePositions(),
        new Pose2d(getPose().getTranslation(), new Rotation2d()) // 向きだけゼロにする
    );
}

  public ChassisSpeeds getChassisSpeeds() {
    return new ChassisSpeeds(0.0, 0.0, 0.0);
  }

  public double getMaxAngularVelocityRadiansPerSec() {
    return maxAngularVelocityRadiansPerSec;
  }

  public double getMaxLinearVelocityMetersPerSec() {
    return maxLinearVelocityMetersPerSec;
  }


  public void configureAutoBuilder() {
    try {
      RobotConfig.fromGUISettings();
      System.out.println("Apply GUI settings");
      new RobotConfig(
        30,
        6.883,
        new ModuleConfig(0.05408, 4.110, 1.1, DCMotor.getNEO(1), 6.75, 40.0, 1),
        new Translation2d(trackWidhMeters / 2, trackWidhMeters / 2),
        new Translation2d(trackWidhMeters / 2, -trackWidhMeters / 2),
        new Translation2d(-trackWidhMeters / 2, trackWidhMeters / 2),
        new Translation2d(-trackWidhMeters / 2, -trackWidhMeters / 2)
      );
    }
    catch (Exception e) {
      e.printStackTrace();
      System.out.println("GUI settings not found");
    }
  }
}

