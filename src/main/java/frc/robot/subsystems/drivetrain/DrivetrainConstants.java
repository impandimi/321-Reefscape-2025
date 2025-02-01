/* (C) Robolancers 2025 */
package frc.robot.subsystems.drivetrain;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.RobotBase;

@Logged
public class DrivetrainConstants {
  public record AutoGains(double kP, double kI, double kD) {}

  public static final AutoGains kTranslationGains =
      RobotBase.isReal()
          ? new AutoGains(5, 0, 0) // real
          : new AutoGains(5, 0, 0); // sim

  public static final AutoGains kHeadingGains =
      RobotBase.isReal()
          ? new AutoGains(5, 0, 0) // real
          : new AutoGains(5, 0, 0); // sim

  public static final AutoGains tuneTranslationGains = new AutoGains(0, 0, 0); // isn't used yet
  public static final AutoGains tuneHeadingGains = new AutoGains(0, 0, 0); // for heading controller

  public static final double kDriveDeadband = 0.03;
  public static final double kRotationDeadband = 0.03;
  public static final double kMaxAngularVelocity = Math.PI * 6;
  public static final double kMaxLinearVelocity =
      5.0; // TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);

  public static final Time kLoopDt = Seconds.of(0.02);
}
