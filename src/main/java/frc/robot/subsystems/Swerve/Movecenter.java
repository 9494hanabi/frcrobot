package frc.robot.subsystems.Swerve;
// 必要なインポート例

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
// import org.photonvision.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

// import java.lang.LiveStackFrame.PrimitiveSlot;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonPoseEstimator;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import com.ctre.phoenix6.swerve.SwerveModule;
import com.studica.frc.AHRS;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SerialPort;

public class Movecenter extends SubsystemBase{
    private PhotonCamera photonCamera;
    private AHRS navX;
    private Joystick joystick;
    private SwerveDriveKinematics kinematics;
    private Module[] swerveModules; // 各スワーブモジュール
    private Transform3d robotToCam;
    private Pose3d tag19FieldPose; // AprilTag 19のフィールド上のPose


    public Movecenter(PhotonCamera photonCamera, AHRS navX, Joystick driverController,
            SwerveDriveKinematics kinematics, Module[] modules, Transform3d robotToCam,
            Optional<Pose3d> tag19FieldPoseOpt) {
        //TODO Auto-generated constructor stub
    }

    /**
     * コンストラクタ
     * @param photonCamera PhotonCameraインスタンス
     * @param navX NavX（AHRS）インスタンス
     * @param joystick 操作用ジョイスティック
     * @param kinematics スワーブドライブ運動学
     * @param swerveModules 各スワーブモジュール群
     * @param robotToCam ロボット→カメラの変換
     * @param tag19FieldPoseOpt AprilTag 19のフィールド上Pose（Optional）
     */
    public void MoveCenter(PhotonCamera photonCamera, AHRS navX, Joystick joystick,
                      SwerveDriveKinematics kinematics, Module[] swerveModules,
                      Transform3d robotToCam, Optional<Pose3d> tag19FieldPoseOpt) {
        this.photonCamera = photonCamera;
        this.navX = navX;
        this.joystick = joystick;
        this.kinematics = kinematics;
        this.swerveModules = swerveModules;
        this.robotToCam = robotToCam;
        if (tag19FieldPoseOpt.isPresent()) {
            this.tag19FieldPose = tag19FieldPoseOpt.get();
        } else {
            System.out.println("Tag 19 のフィールド上の位置が見つからないため、自動アライメントを無効化します。");
            this.tag19FieldPose = null;
        }
    }

    /**
     * 毎周期呼ばれる実行メソッド
     * ボタン10が押され、かつAprilTag 19が視界にあれば自動位置合わせ処理を実行
     */
    public void execute() {
        if (joystick.getRawButton(10) && tag19FieldPose != null) {
            PhotonPipelineResult result = photonCamera.getLatestResult();
            if (result.hasTargets()) {
                PhotonTrackedTarget bestTarget = result.getBestTarget();
                if (bestTarget.getFiducialId() == 19) {
                    // カメラからタグへの変換（カメラ座標系での相対位置）
                    Transform3d camToTag = bestTarget.getBestCameraToTarget();
                    // タグのフィールド上Poseからカメラのフィールド上Poseを計算
                    Pose3d cameraFieldPose = tag19FieldPose.transformBy(camToTag.inverse());
                    // カメラのPoseからロボット中心のフィールド上Poseを算出（robotToCam変換を補正）
                    Pose3d robotFieldPose = cameraFieldPose.transformBy(robotToCam.inverse());
                    // ロボットとタグの位置誤差をフィールド座標系で計算
                    double errorX = tag19FieldPose.getX() - robotFieldPose.getX();
                    double errorY = tag19FieldPose.getY() - robotFieldPose.getY();

                    // 簡易比例制御（必要に応じてゲイン調整）
                    double kP = 1.0;
                    double vxField = errorX * kP;
                    double vyField = errorY * kP;
                    double omega = 0.0;  // 回転制御が必要ならここを調整

                    // NavXの角度を用いてフィールド相対の速度をロボット座標系に変換
                    Rotation2d robotAngle = navX.getRotation2d();
                    ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(vxField, vyField, omega, robotAngle);

                    // 各スワーブモジュールの目標状態を計算
                    SwerveModuleState[] moduleStates = kinematics.toSwerveModuleStates(chassisSpeeds);
                    SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, Module.wheelMaxLinearVelocity);

                    // 各モジュールへ状態を適用
                    for (int i = 0; i < moduleStates.length; i++) {
                        swerveModules[i].run(moduleStates[i]);
                    }
                }
            }
        }
    }
}