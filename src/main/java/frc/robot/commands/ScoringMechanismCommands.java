/* (C) Robolancers 2025 */
package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.algaeIntakePivot.AlgaeIntakePivot;
import frc.robot.subsystems.algaeIntakePivot.AlgaeIntakePivotConstants;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.elevatorarm.ElevatorArm;
import java.util.Set;

public class ScoringMechanismCommands {

  // moves the entire elevator+arm superstructure to a desired state; this should be the go-to way
  // of moving the superstructure, aside from the default subsystem commands
  public static Command goToSetpoint(
      Elevator elevator, ElevatorArm arm, CoralScorerSetpoint setpoint) {
    return elevator
        .goToHeight(() -> setpoint.getElevatorHeight())
        .alongWith(arm.goToAngle(() -> setpoint.getArmAngle()));
  }

  public static Command algaeToSetpoint(Elevator elevator, AlgaeIntakePivot pivot, Angle angle) {
    return Commands.defer(
        () -> {
          Angle currentAngle = pivot.getAngle();
          if (elevator.getHeight().compareTo(ElevatorConstants.kElevatorDangerHeight) < 0
              && // elevator in danger zone
              ((currentAngle.compareTo(AlgaeIntakePivotConstants.kMinFreeAngle) > 0
                      && // we're transitioning between
                      angle.compareTo(AlgaeIntakePivotConstants.kMinFreeAngle) < 0)
                  || (currentAngle.compareTo(AlgaeIntakePivotConstants.kMinFreeAngle) < 0
                      && angle.compareTo(AlgaeIntakePivotConstants.kMinFreeAngle) > 0))) {
            return elevator
                .goToHeight(() -> ElevatorConstants.kElevatorDangerHeight.plus(Meters.of(0.1)))
                .alongWith(pivot.goToAngle(() -> angle))
                .until(pivot::atSetpoint)
                .andThen(pivot.goToAngle(() -> angle));
          }
          return pivot.goToAngle(() -> angle);
        },
        Set.of(pivot));
  }

  public enum CoralScorerSetpoint {
    // TODO: determine angles empirically
    NEUTRAL(Inches.of(0), Degrees.of(0)),
    FEED_CORAL(Inches.of(0), Degrees.of(0)),
    L1(Inches.of(0), Degrees.of(0)),
    L2(Inches.of(0), Degrees.of(0)),
    L3(Inches.of(0), Degrees.of(0)),
    L4(Inches.of(0), Degrees.of(0));

    private Distance elevatorHeight; // the height of the elevator to got
    private Angle armAngle; // the angle the arm should go to

    CoralScorerSetpoint(Distance elevatorHeight, Angle armAngle) {
      this.armAngle = armAngle;
      this.elevatorHeight = elevatorHeight;
    }

    public Distance getElevatorHeight() {
      return elevatorHeight;
    }

    public Angle getArmAngle() {
      return armAngle;
    }
  }
}
