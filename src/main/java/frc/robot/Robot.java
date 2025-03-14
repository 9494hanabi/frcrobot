// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;

// import static edu.wpi.first.wpilibj2.command.Commands.none;

// import java.util.function.Consumer;
import java.util.Optional;

import org.photonvision.PhotonCamera;

// import org.opencv.features2d.FlannBasedMatcher;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.TimedRobot;
//import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.Elevatorsub;
import frc.robot.subsystems.Goal;
import frc.robot.subsystems.Climbsub;
import frc.robot.commands.Elevatorcom;
import frc.robot.subsystems.Climbsub;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.subsystems.Shoot;
import frc.robot.subsystems.Swerve.*;
import frc.robot.commands.Enter1second;
import frc.robot.commands.Turn1second;

//pathplannerのいんぽーと
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.controllers.PathFollowingController;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.util.DriveFeedforwards;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

// import com.pathplanner.lib.drive.DriveFeedforwards;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;


import com.studica.frc.AHRS;


// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.kinematics.ChassisSpeeds;

// import com.ctre.phoenix6.hardware.TalonFX;
// import com.ctre.phoenix6.configs.TalonFXConfiguration;
// import com.ctre.phoenix6.controls.PositionVoltage; 
// import com.ctre.phoenix6.signals.NeutralModeValue;
/**
 * The methods in this class are called automatically corresponding to each mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the package after creating
 * this project, you must also update the Main.java file in the project.
 */
public class Robot extends TimedRobot {
  private final SendableChooser<Command> chooser = new SendableChooser<>();
  //private Joystick joystick;
  //Swerve swerve = new Swerve(0);
  Joystick driverController = new Joystick(0);
  // Joystick elevatorcontroller  = new Joystick(1);

  private RobotContainer robotContainer;  // 🚀 RobotContainerを追加！
  Swerve swerve = new Swerve(0);
  Module module;

  Elevatorsub elevator = new Elevatorsub();
  Elevatorcom elevatorCom = new Elevatorcom(elevator, driverController,1,2);
  Climbsub climbsub = new Climbsub(driverController);
  Goal goal = new Goal(driverController);
  Shoot shoot = new Shoot(driverController);
  private Command m_autonomousCommand;
  Movecenter moveCenter;



  /**
   * This method is run when the robot is first started up and should be used for any initialization
   * code.
   */
  public Robot() {
    //chooser.setDefaultOption("Default Auto", none());
    SmartDashboard.putData("Auto choices", chooser);


    // ドライブベースのデフォルトコマンドセット
    swerve.setDefaultCommand(
        swerve.teleopDrive(
            // WPILibの数学は+X=前、+Y=左なんだけど、コントローラーのスティックは+X=右、+Y=下なの。
            // あとスティックドリフトはありだからMathUtil.applyDeadbandで小さい入力は無視
            () ->MathUtil.applyDeadband(driverController.getRawAxis(1), 0.05), // Y軸（前後）
            () ->MathUtil.applyDeadband(driverController.getRawAxis(0), 0.05), // X軸（左右）
            () ->MathUtil.applyDeadband(-driverController.getRawAxis(4), 0.05)
            )); // Z軸（回転）
  }

  

  @Override
  public void robotInit() {
    PhotonCamera photonCamera = new PhotonCamera("XX");
    Transform3d robotToCam = new Transform3d(
      new Translation3d(0,0,1.5),
      new Rotation3d(0,0,0)
    );
              // Robot の初期化時に AutoBuilder の設定を呼び出す
              swerve.configureAutoBuilder();// コマンドが終了したらログを出力する
          
              swerve.resetHeading(); // NavX の角度リセット
          
              robotContainer = new RobotContainer(); // 🎮 ボタン設定を初期化！
          
              AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);
              Optional<Pose3d> tag19FieldPoseOpt = aprilTagFieldLayout.getTagPose(19);

              AHRS navX = swerve.navx;
              SwerveDriveKinematics kinematics = swerve.kinematics;
              Module[] modules;

              
          
              //デバッグ
              // コマンドが終了するたびにデバッグ情報を出す
              CommandScheduler.getInstance().onCommandFinish(command -> {
                System.out.println("✅ コマンド終了: " + command.getName());
          
                // もし実行中のコマンドが何もなければメッセージを出力
                if (!CommandScheduler.getInstance().isScheduled(command)) {
                  System.out.println("🎉 全てのコマンドが完了しました！ 🎉");
                }
              });
    moveCenter = new Movecenter(photonCamera, navX, driverController, kinematics, modules, robotToCam, tag19FieldPoseOpt);
  }
      
      
      
        @Override
  public void teleopInit() {
    elevator.resetPosition();
    elevatorCom.resetTarget();
  }

  @Override
  public void teleopPeriodic() {
    moveCenter.execute();
    climbsub.Climb();
    elevatorCom.execute();
    goal.goal();
    shoot.ShootBall();
  }

  @Override
  public void robotPeriodic() {
    // // Runs the Scheduler. This is responsible for polling buttons, adding newly-scheduled commands,
    // // running already-scheduled commands, removing finished or interrupted commands, and running
    // // subsystem periodic() methods. This must be called from the robot's periodic block in order
    // // for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
    
    // // NavX の角度を SmartDashboard に表示
    // // System.out.println("NavX Heading (Degrees)" + swerve.getHeading().getDegrees());
    // // System.out.println("NavX Yaw" + swerve.getyaw());
    // // System.out.println("NavX Connected" + swerve.IsConnected());
    
    // climbsub.Climb();
    // elevatorCom.execute();
    // goal.goal();
    // shoot.ShootBall();


  }


  @Override
  public void autonomousInit() {
    Command selectedAuto = robotContainer.getAutonomousCommand();
    if (selectedAuto != null) {
        selectedAuto.schedule();
    }
    // m_autonomousCommand = m_robotContainer.getAutonomous
    
    // autoCommand = chooser.getSelected(); // 選択したオートを取得
    
    // if (autoCommand == null) {
    //     autoCommand = AutoBuilder.buildAuto("New Auto"); // PathPlannerのオートをロード
    //     System.out.println("🚀 オートコマンドをスケジュール: " + autoCommand.getName());
    // } else {
    //   System.out.println("⚠ WARNING: オートコマンドが選択されていません！");
    // }

    // if (autoCommand != null) {
    //     autoCommand.schedule(); // コマンドをスケジュール
    // }
  }

  // @Override
  // public void autonomousExit() {
  //   autoCommand.cancel();
  // }
  // @Override
  // public void autonomousPeriodic() {
  //   //CommandScheduler.getInstance().run(); // コマンドを更新
  // }
}