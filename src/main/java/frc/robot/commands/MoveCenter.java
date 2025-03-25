package frc.robot.commands;

import java.util.Optional;
import frc.robot.subsystems.Swerve.*;
import frc.robot.subsystems.*;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class MoveCenter extends Command {
    private final Swerve swerve;
    private final VisionSubsystem vision;
    
    private final PIDController forwardPID;
    private final PIDController strafePID;
    private final PIDController anglePID;

    private final int targetTagID = 20;
    private final double stopDistance = 0.5;

    public MoveCenter(Swerve swerveSubsystem, VisionSubsystem visionSubsystem) {
        this.swerve = swerveSubsystem;
        this.vision = visionSubsystem;
        addRequirements(swerveSubsystem);

        forwardPID = new PIDController(0.2, 0.0, 0.01);
        strafePID = new PIDController(0.2, 0, 0.01);
        anglePID = new PIDController(0.1, 0.0, 0.005);
    }

    @Override
    public void execute() {
        Optional<Pose2d> tagPoseOpt = vision.getTagPose2d(targetTagID);
        if (tagPoseOpt.isPresent()) {
            Pose2d tagPose = tagPoseOpt.get();
            Rotation2d tagRot = tagPose.getRotation();
            
            Pose2d currentPose = swerve.getPose();

            double targetX = tagPose.getX() - stopDistance * Math.cos(tagRot.getRadians());
            double targetY = tagPose.getY() - stopDistance * Math.sin(tagRot.getRadians());
            Rotation2d targetRot = tagRot.plus(Rotation2d.fromDegrees(180));

            double errorX = targetX - currentPose.getX();
            double errorY = targetY - currentPose.getY();
            double errorTheta = targetRot.minus(currentPose.getRotation()).getRadians();
            errorTheta = Math.atan2(Math.sin(errorTheta), Math.cos(errorTheta)); // -π ~ π の範囲に正規化

            double forwardOp = forwardPID.calculate(errorX, 0);
            double strafeOp = strafePID.calculate(errorY, 0);
            double turnOp = anglePID.calculate(errorTheta, 0);

            ChassisSpeeds optChassisSpeeds = new ChassisSpeeds(-forwardOp, strafeOp, turnOp);

            System.out.println("optChassis : " + optChassisSpeeds);
            swerve.drive(optChassisSpeeds, null); // フィールドオリエンテーションを考慮
        }
    }

    @Override
    public boolean isFinished() {
        // 常時実行する場合は false を返す（必要に応じて条件を追加してください）
        return false;
    }
}
