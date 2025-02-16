/* (C) Robolancers 2025 */
package frc.robot.subsystems.vision;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.robot.RobotConstants;
import frc.robot.subsystems.vision.VisionConstants.CameraConfig;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

@Logged
public class Camera {
  // for logging
  private final String name;

  private final CameraUsage usage;

  private final PhotonCamera camera;

  private final PhotonPoseEstimator poseEstimator;

  // for logging
  private VisionEstimate latestValidEstimate;

  public Camera(CameraConfig config, PhotonCamera camera) {
    this.name = config.cameraName();

    this.usage = config.usage();

    this.camera = camera;

    this.poseEstimator =
        new PhotonPoseEstimator(
            RobotConstants.kAprilTagFieldLayout,
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            config.robotToCamera());
  }

  @NotLogged
  public VisionEstimate tryLatestEstimate() {
    if (!camera.isConnected()) return null;

    final var unreadResults = camera.getAllUnreadResults();

    if (unreadResults.isEmpty()) return null;

    final var latestResult = unreadResults.get(unreadResults.size() - 1);

    if (!latestResult.hasTargets()) return null;

    final var estimate = poseEstimator.update(latestResult);

    return estimate
        .map(
            photonEst -> {
              if (!isPoseValid(photonEst)) return null;
              final var visionEst =
                  new VisionEstimate(photonEst, calculateStdDevs(latestResult), name, usage);
              latestValidEstimate = visionEst;
              return visionEst;
            })
        .orElse(null);
  }

  private static boolean isPoseValid(EstimatedRobotPose pose) {
    boolean isInLowerXLimit = -2.5 <= pose.estimatedPose.getX();
    boolean isInUpperXLimit =
        pose.estimatedPose.getX() <= RobotConstants.kAprilTagFieldLayout.getFieldLength() + 2.5;
    boolean isInLowerYLimit = -2.5 <= pose.estimatedPose.getY();
    boolean isInUpperYLimit =
        pose.estimatedPose.getY() <= RobotConstants.kAprilTagFieldLayout.getFieldWidth() + 2.5;
    return isInLowerXLimit && isInUpperXLimit && isInLowerYLimit && isInUpperYLimit;
  }

  // could be absolute nonsense, open to tuning constants for each robot camera config
  // assumes `result` has targets
  @NotLogged
  private Matrix<N3, N1> calculateStdDevs(PhotonPipelineResult result) {
    final double stdDev = 1 / result.getTargets().size();

    // weighted average by ambiguity
    final double avgTargetDistance =
        result.getTargets().stream()
                .mapToDouble(
                    t -> {
                      final double bestCameraToTargetDistance =
                          t.getBestCameraToTarget()
                              .getTranslation()
                              .getDistance(new Translation3d());
                      final double altCameraToTargetDistance =
                          t.getAlternateCameraToTarget()
                              .getTranslation()
                              .getDistance(new Translation3d());

                      // if (this.usage == CameraUsage.REEF && t.getFiducialId() == 1) {
                      System.out.println(
                          bestCameraToTargetDistance
                              + " "
                              + altCameraToTargetDistance
                              + " "
                              + t.getPoseAmbiguity());
                      // }
                      if (t.getPoseAmbiguity() > 0.45) return 10000;

                      // return (t.getPoseAmbiguity() + 0.8) * bestCameraToTargetDistance;
                      return bestCameraToTargetDistance;
                    })
                .reduce(0.0, Double::sum)
            / (result.getTargets().size());

    final double translationStdDev =
        VisionConstants.kTranslationStdDevCoeff * avgTargetDistance * avgTargetDistance * stdDev;
    final double rotationStdDev =
        VisionConstants.kRotationStdDevCoeff * avgTargetDistance * avgTargetDistance * stdDev;

    return VecBuilder.fill(translationStdDev, translationStdDev, rotationStdDev);
  }
}
