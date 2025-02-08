/* (C) Robolancers 2025 */
package frc.robot;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.SuperstructureVisualizer;
import frc.robot.subsystems.algaeIntakePivot.AlgaeIntakePivot;
import frc.robot.subsystems.algaeIntakeRollers.AlgaeIntakeRollers;
import frc.robot.subsystems.coralendeffector.CoralEndEffector;
import frc.robot.subsystems.drivetrain.SwerveDrive;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevatorarm.ElevatorArm;

@Logged
public class RobotContainer {

  private SwerveDrive drivetrain = SwerveDrive.create();
  private AlgaeIntakePivot algaePivot = AlgaeIntakePivot.create();
  private AlgaeIntakeRollers algaeRollers = AlgaeIntakeRollers.create();
  private CoralEndEffector coralEndEffector = CoralEndEffector.create();
  private ElevatorArm elevatorArm = ElevatorArm.create();
  private Elevator elevator = Elevator.create();

  private CommandXboxController driver = new CommandXboxController(0);
  private CommandXboxController manipulator = new CommandXboxController(1);

  private SuperstructureVisualizer stateVisualizer =
      new SuperstructureVisualizer(
          () -> elevator.getHeight(), () -> elevatorArm.getAngle(), () -> algaePivot.getAngle());

  public RobotContainer() {

    configureBindings();
  }

  private void configureBindings() {}

  public Command getAutonomousCommand() {
    return Commands.none();
  }
}
