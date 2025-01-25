/* (C) Robolancers 2025 */
package frc.robot.subsystems.vision;

import frc.robot.subsystems.vision.VisionConstants.CameraConfig;
import frc.robot.util.VirtualSubsystem;

public class Vision extends VirtualSubsystem {

  private final PoseEstimatorCamera estimatorCamera;

  public Vision(CameraConfig estimatorConfig) {
    this.estimatorCamera = new PoseEstimatorCamera(estimatorConfig);
  }

  @Override
  public void periodic() {
    estimatorCamera.update();
  }
}
