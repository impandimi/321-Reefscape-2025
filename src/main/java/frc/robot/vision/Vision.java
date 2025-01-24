/* (C) Robolancers 2025 */
package frc.robot.vision;

import frc.robot.util.VirtualSubsystem;
import frc.robot.vision.VisionConstants.CameraConfig;

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
