/* (C) Robolancers 2025 */
package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.algaeIntakePivot.AlgaeIntakePivot;
import frc.robot.subsystems.algaeIntakeRollers.AlgaeIntakeRollers;
import frc.robot.subsystems.coralendeffector.CoralEndEffector;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.elevatorarm.ElevatorArm;

@Logged
public class RobotContainer {

  private AlgaeIntakePivot algaePivot = AlgaeIntakePivot.create();
  private AlgaeIntakeRollers algaeRollers = AlgaeIntakeRollers.create();
  private CoralEndEffector coralEndEffector = CoralEndEffector.create();
  private ElevatorArm elevatorArm = ElevatorArm.create();
  private Elevator elevator = Elevator.create();

  private CommandXboxController driver = new CommandXboxController(0);
  private CommandXboxController manipulator = new CommandXboxController(1);

  public RobotContainer() {

    algaePivot.setDefaultCommand(algaePivot.goToAngle(() -> Degrees.of(-16)));
    elevator.setDefaultCommand(elevator.goToHeight(() -> Meters.of(0.7)));

    // when both are about to collide, move elevator out of the way until the algae pivot is out of
    // the collision zone
    new Trigger(algaePivot::inCollisionZone)
        .and(new Trigger(elevator::inCollisionZone))
        .onTrue(
            elevator
                .goToHeight(() -> ElevatorConstants.kElevatorDangerHeight.plus(Meters.of(0.1)))
                // .withInterruptBehavior(InterruptionBehavior.kCancelIncoming)
                .until(new Trigger(algaePivot::inCollisionZone).negate()));

    configureBindings();
  }

  private void configureBindings() {
    driver.a().whileTrue(algaePivot.goToAngle(() -> Degrees.of(180)));
    driver.b().whileTrue(elevator.goToHeight(() -> Meters.of(1.3)));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
