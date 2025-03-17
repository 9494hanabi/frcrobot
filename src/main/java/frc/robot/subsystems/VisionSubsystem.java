package frc.robot.subsystems;

import java.util.Optional;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class VisionSubsystem extends SubsystemBase{
    private final PhotonCamera photonCamera;

    public VisionSubsystem(String cameraName) {
        photonCamera = new PhotonCamera(cameraName);
    }

    public Optional<Pose2d> getTagPose2d(int targetTagID) {
        PhotonPipelineResult result = photonCamera.getLatestResult();
        if (result.hasTargets()) {
            for (PhotonTrackedTarget target : result.getTargets()) {
                if (target.getFiducialId() == targetTagID) {
                    double distance = 3.0;
                    double yawRadians = Math.cos(target.getYaw());
                    double x = distance * Math.cos(yawRadians);
                    double y = distance * Math.sin(yawRadians);

                    Pose2d tagPose = new Pose2d(x, y, new Rotation2d(0));
                    return Optional.of(tagPose);
                }
            }
        }
        return Optional.empty();
    }
}
