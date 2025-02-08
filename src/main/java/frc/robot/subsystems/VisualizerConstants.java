/* (C) Robolancers 2025 */
package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Distance;
import frc.robot.subsystems.elevator.ElevatorConstants;

public class VisualizerConstants {
  // elevator base relative to front of the robot
  public static final Translation2d elevatorRoot2d =
      new Translation2d(Inches.of(15).in(Meters), Inches.of(1).in(Meters));

  // elevator base in 3d
  public static final Translation3d elevatorRoot3d =
      new Translation3d(elevatorRoot2d.getX(), 0, elevatorRoot2d.getY());

  // middle of elevator (side view) to middle of shoulder joint
  public static final Distance shoulderToElevatorOffset = Inches.of(11.5);

  // arm shoulder joint, top of elevator
  public static final Translation2d armRoot2d =
      elevatorRoot2d.plus(
          new Translation2d(
              -shoulderToElevatorOffset.in(Meters),
              ElevatorConstants.kElevatorMinimumHeight.in(Meters)));

  // arm shoulder joint in 3d
  public static final Translation3d armRoot3d =
      new Translation3d(armRoot2d.getX(), 0, armRoot2d.getY());
}
