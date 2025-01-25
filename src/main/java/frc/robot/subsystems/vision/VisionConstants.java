/* (C) Robolancers 2025 */
package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Transform3d;

public class VisionConstants {
  public static record CameraConfig(String cameraName, Transform3d robotToCamera) {}

  public static final CameraConfig k427CameraConfig = new CameraConfig("camera", new Transform3d());

  public static final AprilTagFieldLayout kAprilTagFieldLayout =
      AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);
}
