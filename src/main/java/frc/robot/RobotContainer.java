/* (C) Robolancers 2025 */
package frc.robot;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.algaeIntakePivot.AlgaeIntakePivot;
import frc.robot.subsystems.algaeIntakeRollers.AlgaeIntakeRollers;

@Logged
public class RobotContainer {

  private AlgaeIntakePivot pivot = AlgaeIntakePivot.create();
  private AlgaeIntakeRollers rollers = AlgaeIntakeRollers.create();

  public RobotContainer() {
    configureBindings();
  }

  private CommandXboxController driverController = new CommandXboxController(0);
  private CommandXboxController manipulatorController = new CommandXboxController(1);

  private void configureBindings() {}

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
