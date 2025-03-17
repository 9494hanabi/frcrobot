// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Optional;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;

import frc.robot.subsystems.Elevatorsub;
import frc.robot.subsystems.Goal;
import frc.robot.subsystems.Climbsub;
import frc.robot.commands.Elevatorcom;
import frc.robot.commands.MoveCenter;
import frc.robot.subsystems.Climbsub;
import frc.robot.subsystems.Shoot;
import frc.robot.subsystems.Swerve.*;
import frc.robot.subsystems.Swerve.Module;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.controllers.PathFollowingController;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.util.DriveFeedforwards;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;

import org.photonvision.PhotonCamera;
import com.studica.frc.AHRS;
/**
 * The methods in this class are called automatically corresponding to each mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the package after creating
 * this project, you must also update the Main.java file in the project.
 */
public class Robot extends TimedRobot {
  private final SendableChooser<Command> chooser = new SendableChooser<>();
  Joystick driverController = new Joystick(0);

  private RobotContainer robotContainer;
  Swerve swerve = new Swerve(0);
  Module[] modules;
  Elevatorsub elevator = new Elevatorsub();
  Elevatorcom elevatorCom = new Elevatorcom(elevator, driverController,1,2);
  Climbsub climbsub = new Climbsub(driverController);
  Goal goal = new Goal(driverController);
  Shoot shoot = new Shoot(driverController);
  MoveCenter moveCenter;
  private Command m_autonomousCommand;

  /**
   * This method is run when the robot is first started up and should be used for any initialization
   * code.
   */
  public Robot() {
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
              // Robot の初期化時に AutoBuilder の設定を呼び出す
              swerve.configureAutoBuilder();// コマンドが終了したらログを出力する
          
              swerve.resetHeading(); // NavX の角度リセット
          
              robotContainer = new RobotContainer(); // 🎮 ボタン設定を初期化！

              
          
              //デバッグ
              // コマンドが終了するたびにデバッグ情報を出す
              CommandScheduler.getInstance().onCommandFinish(command -> {
                System.out.println("✅ コマンド終了: " + command.getName());
          
                // もし実行中のコマンドが何もなければメッセージを出力
                if (!CommandScheduler.getInstance().isScheduled(command)) {
                  System.out.println("🎉 全てのコマンドが完了しました！ 🎉");
                }
              });
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


  }


  @Override
  public void autonomousInit() {
    Command selectedAuto = robotContainer.getAutonomousCommand();
    if (selectedAuto != null) {
        selectedAuto.schedule();
    }
  }
}