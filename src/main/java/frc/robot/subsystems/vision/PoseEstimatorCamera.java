/* (C) Robolancers 2025 */
package frc.robot.subsystems.vision;

import edu.wpi.first.epilogue.Logged;
import frc.robot.subsystems.vision.VisionConstants.CameraConfig;
import java.util.List;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

// abstraction for future multicamera setups
@Logged
public class PoseEstimatorCamera {
  private final String name;

  private final PhotonCamera camera;
  private final PhotonPoseEstimator poseEstimator;

  private EstimatedRobotPose latestEstimate;

  public PoseEstimatorCamera(CameraConfig config) {
    this.name = config.cameraName();

    this.camera = new PhotonCamera(config.cameraName());

    this.poseEstimator =
        new PhotonPoseEstimator(
            VisionConstants.kAprilTagFieldLayout,
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            config.robotToCamera());
  }

  public void update() {
    final List<PhotonPipelineResult> results = camera.getAllUnreadResults();

    final Optional<EstimatedRobotPose> optionalEstimate =
        poseEstimator.update(results.get(results.size() - 1));

    optionalEstimate.ifPresent(
        est -> {
          poseEstimator.setReferencePose(est.estimatedPose);
          latestEstimate = est;
        });
  }

  // for use by drivetrain pose estimator, may consolidate into a robot-wide pose estimator
  public EstimatedRobotPose getLatestEstimate() {
    return latestEstimate;
  }
}
