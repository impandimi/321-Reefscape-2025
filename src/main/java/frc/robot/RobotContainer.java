/* (C) Robolancers 2025 */
package frc.robot;

import static edu.wpi.first.units.Units.Volt;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.elevator.Elevator;

@Logged
public class RobotContainer {
  // TODO: add subsystems here
  // private ElevatorArm elevatorArm = ElevatorArm.create();
  private Elevator elevator = Elevator.create();

  private CommandXboxController driver = new CommandXboxController(0);

  public RobotContainer() {
    // System.out.println(ElevatorConstants.kPositionConversionFactor);
    // TODO: init subsystems here
    // elevatorArm.setDefaultCommand(elevatorArm.runVolts(() -> Volts.of(0)));
    // elevatorArm.setDefaultCommand(elevatorArm.goToAngle(() -> Degrees.of(20)));
    elevator.setDefaultCommand(elevator.setVoltage(() -> Volt.zero()));

    configureBindings();
  }

  private void configureBindings() {
    // driver.a().whileTrue(elevator.goToHeight(() -> ElevatorConstants.kElevatorStartingHeight));
    // driver.b().whileTrue(elevator.goToHeight(() ->
    // ElevatorConstants.kElevatorStartingHeight.times(1.5)));
    // driver.y().whileTrue(elevator.goToHeight(() ->
    // ElevatorConstants.kElevatorStartingHeight.times(2)));
    // driver.x().whileTrue(elevator.goToHeight(() ->
    // ElevatorConstants.kElevatorStartingHeight.times(3)));
    driver.b().whileTrue(elevator.tune());
    driver.a().whileTrue(elevator.setVoltage(() -> Volt.of(-1)));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
