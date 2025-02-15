/* (C) Robolancers 2025 */
package frc.robot.subsystems.vision;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.RobotConstants;
import frc.robot.subsystems.vision.VisionConstants.CameraConfig;
import java.util.ArrayList;
import java.util.List;
import java.util.Objects;
import java.util.function.Supplier;
import java.util.stream.Stream;
import org.photonvision.PhotonCamera;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.simulation.VisionTargetSim;

@Logged
public class VisionIOSim implements VisionIO {
  @NotLogged private final VisionSystemSim sim;

  private final List<CameraSim> cameras;

  private final Supplier<Pose2d> robotPoseSupplier;

  @NotLogged private final List<VisionTargetSim> targets;

  public VisionIOSim(Supplier<Pose2d> robotPoseSupplier, CameraConfig... configs) {
    this.sim = new VisionSystemSim("main");
    sim.addAprilTags(RobotConstants.kAprilTagFieldLayout);

    this.cameras =
        Stream.of(configs)
            .map(
                config -> {
                  final var cameraSim =
                      new PhotonCameraSim(
                          new PhotonCamera(config.cameraName()), config.simProperties());
                  sim.addCamera(cameraSim, config.robotToCamera());
                  return new CameraSim(config, cameraSim, robotPoseSupplier);
                })
            .toList();

    this.robotPoseSupplier = robotPoseSupplier;

    this.targets = new ArrayList<>(sim.getVisionTargets());
  }

  @Override
  public VisionEstimate[] getLatestEstimates() {
    sim.update(robotPoseSupplier.get());

    return cameras.stream()
        .map(camera -> camera.tryLatestEstimate(targets))
        .filter(Objects::nonNull)
        .toArray(VisionEstimate[]::new);
  }
}
