package frc.robot.commands;

import java.util.Optional;
import frc.robot.subsystems.Swerve.*;
import frc.robot.subsystems.*;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class MoveCenter extends Command{
    //コマンド内のメンバ変数
    private Swerve swerve = new Swerve();
    private VisionSubsystem vision;  
        private final PIDController forwardPID = new PIDController(1, 0, 0);
        private final PIDController strafePID = new PIDController(1,0,0);
        private final PIDController anglePID = new PIDController(1,0,0);
    
        //目標タグの設定＆停止オフセット設定
        private final int targetTagID = 19;
        private final double stopDistance = 0.5; //単位：m(メートル)
    
        public MoveCenter(Swerve swerveSubsystem) {
            setSwerve(swerveSubsystem);
            addRequirements(swerveSubsystem);
        }
    
        public void setSwerve(Swerve swerveSubsystem) {
            this.swerve = swerveSubsystem;
        }
        public void setVision(VisionSubsystem visionSubsystem) {
            this.vision = visionSubsystem;
        }

    public Swerve getSwerve() {
        return this.swerve;
    }

    public VisionSubsystem getVision() {
        return this.vision;
    }

    @Override
    public void execute() {
        Optional<Pose2d> tagPopseOpt = vision.getTagPose2d(targetTagID);
        if (tagPopseOpt.isPresent()) {
            Pose2d tagPose = tagPopseOpt.get();
            Rotation2d tagRot = tagPose.getRotation();
            Pose2d targetPose = new Pose2d(
                tagPose.getX() - stopDistance * Math.cos(tagRot.getRadians()),
                tagPose.getY() - stopDistance * Math.sin(tagRot.getRadians()),
                tagRot.plus(Rotation2d.fromDegrees(180))
            );

            Pose2d currentPose = swerve.getPose();
            Pose2d errorPose = targetPose.relativeTo(currentPose);
            double errorX = errorPose.getX();
            double errorY = errorPose.getY();
            double errorYaw = errorPose.getRotation().getRadians();

            double forwardOp = forwardPID.calculate(errorX, 0);
            double strafeOp = strafePID.calculate(errorY, 0);
            double turnOp = anglePID.calculate(errorYaw, 0);

            ChassisSpeeds optChassisSpeeds = new ChassisSpeeds(forwardOp, strafeOp, turnOp);

            swerve.drive(optChassisSpeeds, null);
        }
        else {
            swerve.drive(null, null);
        }
    }
}