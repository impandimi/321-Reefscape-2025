/* (C) Robolancers 2025 */
package frc.robot;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.elevatorarm.ElevatorArm;

@Logged
public class RobotContainer {
  // TODO: add subsystems here
  private ElevatorArm elevatorArm = ElevatorArm.create();

  private CommandXboxController driver = new CommandXboxController(0);

  public RobotContainer() {
    // TODO: init subsystems here
    // elevatorArm.setDefaultCommand(elevatorArm.runVolts(() -> Volts.of(0)));
    elevatorArm.setDefaultCommand(elevatorArm.goToAngle(() -> Degrees.of(20)));

    configureBindings();
  }

  private void configureBindings() {
    driver.a().whileTrue(elevatorArm.goToAngle(() -> Degrees.of(80)));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
