package frc.robot.subsystems.Swerve;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static edu.wpi.first.units.Units.Rotation;

import java.security.Key;
import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.util.DriveFeedforwards;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;
import com.pathplanner.lib.trajectory.PathPlannerTrajectoryState;  
import com.pathplanner.lib.trajectory.SwerveModuleTrajectoryState;

import com.studica.frc.AHRS;

import edu.wpi.first.wpilibj.SPI.Port;
//import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SerialPort;

//import com.fasterxml.jackson.databind.util.ArrayBuilders.DoubleBuilder;

public class Swerve extends SubsystemBase {
  //public static final Object[] Module;
  
    private Module[] modules;
  
    private SwerveDriveKinematics kinematics;
    //NavX
    private final AHRS navx = new AHRS(AHRS.NavXComType.kMXP_SPI);
      // 各モジュールの現在の位置を取得する（例として初期状態ならすべてゼロとする）
//   SwerveModulePosition[] modulePositions = new SwerveModulePosition[] {
//   new SwerveModulePosition(modules[0].getDistanceMeters(), modules[0].getAngle()),
//   new SwerveModulePosition(modules[1].getDistanceMeters(), modules[1].getAngle()),
//   new SwerveModulePosition(modules[2].getDistanceMeters(), modules[2].getAngle()),
//   new SwerveModulePosition(modules[3].getDistanceMeters(), modules[3].getAngle())
// };

  private SwerveDriveOdometry odometry;

  private SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] positions = new SwerveModulePosition[modules.length];
    for (int i = 0; i < modules.length; i++) {
      positions[i] = modules[i].getPosition(); 
    }
    return positions;
  }



  private Joystick joystick;
                                                     
  // ホイールからホイールの幅、メートル
  private final double trackWidthMeters = 0.675;
  private final double trackLengthMeters = 0.675;

  // ドライブベースの最大速度。秒速メートルと秒速ラジアン
  private final double maxLinearVelocityMetersPerSec = Module.wheelMaxLinearVelocity;
  private final double maxAngularVelocityRadiansPerSec =
      Module.wheelMaxLinearVelocity / Math.hypot(trackLengthMeters / 2, trackWidthMeters / 2);

      
  public Swerve(int joystickPort) {
    modules = new Module[] {new Module(0), new Module(1), new Module(2), new Module(3)};
    joystick = new Joystick(joystickPort);

          // +X=前,+Y=左
  kinematics =
    new SwerveDriveKinematics(
        new Translation2d(trackWidthMeters / 2, trackLengthMeters / 2),
        new Translation2d(trackWidthMeters / 2, -trackLengthMeters / 2),
        new Translation2d(-trackWidthMeters / 2, trackLengthMeters / 2),
        new Translation2d(-trackWidthMeters / 2, -trackLengthMeters / 2));


        SwerveModulePosition[] modulePositions = new SwerveModulePosition[] {
          new SwerveModulePosition(modules[0].getDistanceMeters(), modules[0].getAngle()),
          new SwerveModulePosition(modules[1].getDistanceMeters(), modules[1].getAngle()),
          new SwerveModulePosition(modules[2].getDistanceMeters(), modules[2].getAngle()),
          new SwerveModulePosition(modules[3].getDistanceMeters(), modules[3].getAngle())
        };
        
        odometry = new SwerveDriveOdometry(kinematics, navx.getRotation2d(), modulePositions, new Pose2d(0, 0, new Rotation2d(0)));
}


  // private final SwerveDriveOdometry odometry = new SwerveDriveOdometry(
  //   kinematics,navx.getRotation2d(), new Pose2d(0, 0, 0)
  // );
  

// gyro.getRotation2d() は、例えばGyroセンサから取得するロボットの角度
// odometry = new SwerveDriveOdometry(kinematics, navx.getRotation2d(), modulePositions, new Pose2d(0, 0, new Rotation2d(0)));

  

  public Command teleopDrive(DoubleSupplier Xspeed, DoubleSupplier Yspeed, DoubleSupplier Yawspeed) {
    return run(() -> {
      // 現在のロボットの向きを取得
      


      double X = Xspeed.getAsDouble() * maxLinearVelocityMetersPerSec; // 前後 (Y軸)
      double Y = Yspeed.getAsDouble() * maxLinearVelocityMetersPerSec; // 左右 (X軸)
      double Rad = Yawspeed.getAsDouble() * maxAngularVelocityRadiansPerSec; // 回転 (Z軸)
      
      //ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(X,Y,Rad,robotRotation);

            // 🔽 追加: 一定のしきい値以下なら、完全にゼロにする
            if (Math.abs(X) < 0.01) X = 0;
            if (Math.abs(Y) < 0.01) Y = 0;
            if (Math.abs(Rad) < 0.01) Rad = 0;

      drive(new ChassisSpeeds(X,Y,Rad), null);

      // System.out.println("ChasisSpeed:X " + X);
      // System.out.println("ChasisSpeed:Y " + Y);
      // System.out.println("ChasisSpeed:Rad " + Rad);
      //System.out.println("ChasisSpeed:Rotation " + robotRotation);
    });
  }

  public void drive(ChassisSpeeds desiredSpeeds, DriveFeedforwards feedforwards) {

    System.out.println("PathPlanner is driving! Speeds: " 
        + desiredSpeeds.vxMetersPerSecond + ", " 
        + desiredSpeeds.vyMetersPerSecond + ", " 
        + desiredSpeeds.omegaRadiansPerSecond);

    // ターゲットの速度と角度を計算
    SwerveModuleState[] targetModuleSpeeds = kinematics.toSwerveModuleStates(desiredSpeeds);
    // ターゲットの速度があまりに速すぎないように、たまにはスピードを下げる
    SwerveDriveKinematics.desaturateWheelSpeeds(targetModuleSpeeds, Module.wheelMaxLinearVelocity);

    // ターゲットの速度と角度をセットする
    for (int i = 0; i < 4; i++) {
      modules[i].run(targetModuleSpeeds[i]);
    }
  }

  public Command Drive() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'teleopDrive'");
  }


public Object getPose1() {
  throw new UnsupportedOperationException("Unimplemented method 'getPose'");
}

public Object getChassisSpeeds1() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'getChassisSpeeds'");
}

// 正しく現在のロボット位置を返す
public Pose2d getPose() {
  return odometry.getPoseMeters();
}

// resetOdometry は返り値なし（void）にする
public void resetOdometry(Pose2d pose) {
  // 例：odometry をリセットする処理
  odometry.resetPosition(navx.getRotation2d(), getModulePositions(), pose);
}

// 正しい型でロボット相対速度を返す
public ChassisSpeeds getChassisSpeeds() {
  return new ChassisSpeeds(0.0, 0.0, 0.0);
}

ModuleConfig moduleConfig = new ModuleConfig(0.9, 1.0, 0.9,DCMotor.getNEO(1), 6.75,1.0, 1);
//moduleConfig.wheelCOF = 1.0;

public void configureAutoBuilder() {
  AutoBuilder.configure(
    this::getPose,  // ロボットの現在位置を取得
    this::resetOdometry, // オドメトリをリセット
    this::getChassisSpeeds, // 現在のシャーシ速度を取得
    this::drive, // ロボットを動かす関数
    new PPHolonomicDriveController(
        new PIDConstants(1.0, 0.0, 0.0), // X軸のPID
        new PIDConstants(1.0, 0.0, 0.0), // Y軸のPID
        0.1 // 回転の制約
    ),
    new RobotConfig(74.088, 6.883, moduleConfig, maxAngularVelocityRadiansPerSec),
    () -> false,
    this
);
System.out.println("AutoBuilder configured!"); // 確認用
}
}
