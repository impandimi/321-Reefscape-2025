/* (C) Robolancers 2025 */
package frc.robot.subsystems.drivetrain;

import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

public class DrivetrainConstants {
  public static final double kDriveDeadband = 0.03;
  public static final double kRotationDeadband = 0.03;
  public static final double kMaxAngularVelocity =
      RotationsPerSecond.of(Math.PI).in(RadiansPerSecond);
}
