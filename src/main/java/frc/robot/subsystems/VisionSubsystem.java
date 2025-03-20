package frc.robot.subsystems;

import java.util.Optional;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class VisionSubsystem extends SubsystemBase {
    private final PhotonCamera photonCamera;
    private double estimatedDistance = 3.0;  

    public VisionSubsystem(String cameraName) {
        photonCamera = new PhotonCamera(cameraName);
        SmartDashboard.putNumber("Estimated Distance", estimatedDistance);
    }

    public Optional<Pose2d> getTagPose2d(int targetTagID) {
        PhotonPipelineResult result = photonCamera.getLatestResult();
        if (!result.hasTargets()) {
            return Optional.empty();
        }

        PhotonTrackedTarget bestTarget = result.getBestTarget();
        if (bestTarget == null || bestTarget.getFiducialId() != targetTagID) {
            return Optional.empty();
        }

        estimatedDistance = SmartDashboard.getNumber("Estimated Distance", estimatedDistance);
        double yawRadians = Math.toRadians(bestTarget.getYaw());

        double x = estimatedDistance * Math.cos(yawRadians);
        double y = estimatedDistance * Math.sin(yawRadians);

        Pose2d tagPose = new Pose2d(x, y, new Rotation2d(yawRadians));

        SmartDashboard.putNumber("Target X", x);
        SmartDashboard.putNumber("Target Y", y);
        SmartDashboard.putNumber("Target Yaw (deg)", bestTarget.getYaw());

        return Optional.of(tagPose);
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Has Targets", photonCamera.getLatestResult().hasTargets());
        SmartDashboard.putNumber("Estimated Distance", estimatedDistance);
    }
}
