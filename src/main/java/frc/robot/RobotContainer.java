/* (C) Robolancers 2025 */
package frc.robot;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.coralendeffector.CoralEndEffector;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevatorarm.ElevatorArm;

@Logged
public class RobotContainer {
  private CoralEndEffector endEffector = CoralEndEffector.create();
  private ElevatorArm elevatorArm = ElevatorArm.create();
  private Elevator elevator = Elevator.create();

  private CommandXboxController driver = new CommandXboxController(0);
  private CommandXboxController manipulator = new CommandXboxController(1);

  public RobotContainer() {

    configureBindings();
  }

  private void configureBindings() {}

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
