/* (C) Robolancers 2025 */
package frc.robot;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.algaeIntakepivot.AlgaeIntakePivot;
import frc.robot.subsystems.algaeIntakerollers.AlgaeIntakeRollers;

@Logged
public class RobotContainer {

  private AlgaeIntakePivot pivot = AlgaeIntakePivot.create();
  private AlgaeIntakeRollers rollers = AlgaeIntakeRollers.create();

  public RobotContainer() {
    configureBindings();
    pivot.setDefaultCommand(pivot.setMechanismVoltage(Volts.of(0)));
    rollers.setDefaultCommand(rollers.setMechanismVoltage(Volts.of(0)));
  }

  private CommandXboxController driverController = new CommandXboxController(0);
  private CommandXboxController manipulatorController = new CommandXboxController(1);

  private void configureBindings() {
    driverController.a().whileTrue(rollers.tune());
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
