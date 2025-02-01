/* (C) Robolancers 2025 */
package frc.robot;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.algaeIntakeClimb.AlgaeIntakeClimb;
import frc.robot.subsystems.algaeIntakeRollers.AlgaeIntakeRollers;

@Logged
public class RobotContainer {

  private AlgaeIntakeClimb pivot = AlgaeIntakeClimb.create();
  private AlgaeIntakeRollers rollers = AlgaeIntakeRollers.create();

  public RobotContainer() {
    configureBindings();
    pivot.setDefaultCommand(pivot.setMechanismVoltage(Volts.of(0)));
    rollers.setDefaultCommand(rollers.setMechanismVoltage(Volts.of(0)));
  }

  private CommandXboxController driverController = new CommandXboxController(0);
  private CommandXboxController manipulatorController = new CommandXboxController(1);

  private void configureBindings() {
    driverController.leftBumper().whileTrue(pivot.outtakePosition());
    driverController.leftTrigger().whileTrue(rollers.outtake());

    manipulatorController.rightBumper().whileTrue(pivot.intakePosition());
    manipulatorController.rightTrigger().whileTrue(rollers.intake());

    driverController.y().whileTrue(pivot.climbFloorPosition());
    driverController.x().whileTrue(pivot.climb());
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
