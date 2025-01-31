/* (C) Robolancers 2025 */
package frc.robot;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.elevatorarm.ElevatorArm;

@Logged
public class RobotContainer {
  private ElevatorArm elevatorArm = ElevatorArm.create();
  private Elevator elevator = Elevator.create();

  private CommandXboxController driver = new CommandXboxController(0);
  private CommandXboxController manipulator = new CommandXboxController(1);

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    // These are placeholder set points for now, which will later be replaced by actual set points
    // for elevator
    // Right now, goes starting height to maximum height, in incrememnts of one third of the
    // starting height
    driver.a().whileTrue(elevator.goToHeight(() -> ElevatorConstants.kElevatorStartingHeight));
    driver
        .b()
        .whileTrue(
            elevator.goToHeight(
                () -> ElevatorConstants.kElevatorStartingHeight.times(1.33333333333)));
    driver
        .y()
        .whileTrue(
            elevator.goToHeight(
                () -> ElevatorConstants.kElevatorStartingHeight.times(1.66666666666)));
    driver.x().whileTrue(elevator.goToHeight(() -> ElevatorConstants.kElevatorMaximumHeight));
    // driver.b().whileTrue(elevator.tune());
    // driver.a().whileTrue(elevator.setVoltage(() -> Volt.of(-1)));
    driver.leftBumper().whileTrue(elevator.homeEncoder());
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
