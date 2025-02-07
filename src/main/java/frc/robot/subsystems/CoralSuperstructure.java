/* (C) Robolancers 2025 */
package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.coralendeffector.CoralEndEffector;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevatorarm.ElevatorArm;

public class CoralSuperstructure {

  private Elevator elevator;
  private ElevatorArm arm;
  private CoralEndEffector endEffector;

  private CoralScorerSetpoint targetState = CoralScorerSetpoint.FEED_CORAL;

  public CoralSuperstructure(Elevator elevator, ElevatorArm arm, CoralEndEffector endEffector) {
    this.elevator = elevator;
    this.arm = arm;
    this.endEffector = endEffector;
  }

  // moves the entire elevator+arm superstructure to a desired state; this should be the go-to way
  // of moving the superstructure, aside from the default subsystem commands
  public Command goToSetpoint(CoralScorerSetpoint setpoint) {
    return Commands.runOnce(() -> targetState = setpoint)
        .andThen(
            elevator
                .goToHeight(() -> setpoint.getElevatorHeight())
                .alongWith(arm.goToAngle(() -> setpoint.getArmAngle())));
  }

  public boolean atTargetState() {
    return elevator.atSetpoint() && arm.atSetpoint();
  }

  public Command feedCoral() {
    return goToSetpoint(CoralScorerSetpoint.FEED_CORAL).alongWith(endEffector.intakeCoral());
  }

  public Command outtakeCoral() {
    return endEffector.outtakeCoral();
  }

  public CoralScorerSetpoint getTargetState() {
    return targetState;
  }

  public enum CoralScorerSetpoint {
    // TODO: determine angles empirically
    NEUTRAL(Inches.of(0), Degrees.of(-35)),
    FEED_CORAL(Inches.of(0), Degrees.of(0)),
    L1(Inches.of(0), Degrees.of(0)),
    L2(Inches.of(0), Degrees.of(0)),
    L3(Inches.of(0), Degrees.of(0)),
    L4(Inches.of(0), Degrees.of(0)),
    ALGAE_LOW(Inches.of(0), Degrees.of(0)),
    ALGAE_HIGH(Inches.of(0), Degrees.of(0));

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
