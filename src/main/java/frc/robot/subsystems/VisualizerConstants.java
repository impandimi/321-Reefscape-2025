/* (C) Robolancers 2025 */
package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Angle;
import frc.robot.subsystems.elevator.ElevatorConstants;

public class VisualizerConstants {
  // where the base of the elevator is
  public static final Translation2d elevatorRoot2d =
      new Translation2d(Inches.of(15).in(Meters), Inches.of(1).in(Meters));
  public static final Translation3d elevatorRoot3d =
      new Translation3d(elevatorRoot2d.getX(), 0, elevatorRoot2d.getY());
  public static final Angle kElevatorAngle = Degrees.of(90);

  public static final Translation2d armRoot2d =
      elevatorRoot2d.plus(
          new Translation2d(
              ElevatorConstants.kElevatorMinimumHeight.in(Meters),
              new Rotation2d(kElevatorAngle.in(Radians))));
  public static final Translation3d armRoot3d =
      new Translation3d(armRoot2d.getX(), 0, armRoot2d.getY());
}
