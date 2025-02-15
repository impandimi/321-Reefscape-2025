/* (C) Robolancers 2025 */
package frc.robot.subsystems.vision;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Milliseconds;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.units.measure.Time;
import org.photonvision.simulation.SimCameraProperties;

public class VisionConstants {
  public static final double kTranslationStdDevCoeff = 1.0;
  public static final double kRotationStdDevCoeff = 1.0;

  public static record CameraConfig(
      String cameraName,
      CameraType type,
      Transform3d robotToCamera,
      // sim properties
      int resolutionWidth,
      int resolutionHeight,
      Rotation2d fovDiag,
      double avgErrorPx,
      double errorStdDevPx,
      Time exposureTime,
      double fps,
      Time avgLatency,
      Time latencyStdDev) {
    public SimCameraProperties simProperties() {
      final var simProps = new SimCameraProperties();

      simProps.setCalibration(resolutionWidth, resolutionHeight, fovDiag);
      simProps.setCalibError(avgErrorPx, errorStdDevPx);

      simProps.setExposureTimeMs(exposureTime.in(Milliseconds));
      simProps.setFPS(fps);

      simProps.setAvgLatencyMs(avgLatency.in(Milliseconds));
      simProps.setLatencyStdDevMs(latencyStdDev.in(Milliseconds));

      return simProps;
    }
  }

  private static final Transform3d k427CameraMountTransform =
      new Transform3d(
          Meters.of(-0.27),
          Meters.zero(),
          Meters.zero(),
          new Rotation3d(Degrees.zero(), Degrees.of(30), Degrees.of(180)));

  public static final CameraConfig kUSBCameraConfig =
      new CameraConfig(
          "USB_Camera",
          CameraType.GENERAL,
          k427CameraMountTransform,
          1280,
          720,
          Rotation2d.fromDegrees(70),
          // TODO: find actual values for this from calibration in Photon Client
          // calibration file does not show these values and config.json mentioned in docs appears
          // inaccessible
          0,
          0,
          Seconds.zero(),
          30,
          Seconds.zero(),
          Seconds.zero());
}
