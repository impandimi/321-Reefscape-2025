/* (C) Robolancers 2025 */
package frc.robot.util;

import static edu.wpi.first.units.Units.Degree;

import edu.wpi.first.epilogue.CustomLoggerFor;
import edu.wpi.first.epilogue.logging.ClassSpecificLogger;
import edu.wpi.first.epilogue.logging.EpilogueBackend;
import edu.wpi.first.units.measure.Angle;

// For creating a custom logger, refer to
// https://docs.wpilib.org/en/stable/docs/software/telemetry/robot-telemetry-with-annotations.html#logging-third-party-data
@CustomLoggerFor(Angle.class)
public class AngleLogger extends ClassSpecificLogger<Angle> {

  public AngleLogger() {
    super(Angle.class);
  }

  @Override
  protected void update(EpilogueBackend dataLogger, Angle object) {
    dataLogger.log("", object.in(Degree));
  }
}
