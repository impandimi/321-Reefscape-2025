/* (C) Robolancers 2025 */
package frc.robot.subsystems.vision;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.robot.RobotConstants;
import frc.robot.subsystems.vision.VisionConstants.CameraConfig;
import java.util.List;
import java.util.function.Supplier;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.VisionTargetSim;
import org.photonvision.targeting.PhotonPipelineResult;

@Logged
public class CameraSim {
  private final CameraType type;

  @NotLogged private final PhotonCameraSim camera;

  @NotLogged private final PhotonPoseEstimator poseEstimator;

  private final Supplier<Pose3d> cameraPoseSupplier;

  // for logging
  private VisionEstimate latestValidEstimate;

  public CameraSim(
      CameraConfig config, PhotonCameraSim camera, Supplier<Pose2d> robotPoseSupplier) {
    this.type = config.type();

    this.camera = camera;

    this.poseEstimator =
        new PhotonPoseEstimator(
            RobotConstants.kAprilTagFieldLayout,
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            config.robotToCamera());

    this.cameraPoseSupplier =
        () -> new Pose3d(robotPoseSupplier.get()).plus(config.robotToCamera());
  }

  @NotLogged
  public VisionEstimate tryLatestEstimate(List<VisionTargetSim> targets) {
    final var latestResult =
        camera.process(camera.prop.estLatencyMs(), cameraPoseSupplier.get(), targets);

    if (!latestResult.hasTargets()) return null;

    final var estimate = poseEstimator.update(latestResult);

    return estimate
        .map(
            photonEst -> {
              final var visionEst =
                  new VisionEstimate(photonEst, calculateStdDevs(latestResult), type);
              latestValidEstimate = visionEst;
              return visionEst;
            })
        .orElse(null);
  }

  // could be absolute nonsense, open to tuning constants for each robot camera config
  // assumes `result` has targets
  @NotLogged
  private Matrix<N3, N1> calculateStdDevs(PhotonPipelineResult result) {
    final double stdDev = result.metadata.getLatencyMillis() / result.getTargets().size();

    final double rotationStdDev = VisionConstants.kRotationStdDevCoeff * stdDev;

    final Translation3d robotPose = new Translation3d();

    // weighted average by ambiguity
    final double avgTargetDistance =
        result.getTargets().stream()
                .mapToDouble(
                    t -> {
                      final double bestCameraToTargetDistance =
                          t.getBestCameraToTarget().getTranslation().getDistance(robotPose);
                      final double altCameraToTargetDistance =
                          t.getAlternateCameraToTarget().getTranslation().getDistance(robotPose);

                      return t.getPoseAmbiguity()
                          * (bestCameraToTargetDistance + altCameraToTargetDistance);
                    })
                .reduce(0.0, Double::sum)
            / (2.0 * result.getTargets().size());

    final double translationStdDev =
        VisionConstants.kTranslationStdDevCoeff * avgTargetDistance * stdDev;

    return VecBuilder.fill(translationStdDev, translationStdDev, rotationStdDev);
  }
}
