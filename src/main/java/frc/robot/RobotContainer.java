/* (C) Robolancers 2025 */
package frc.robot;

import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.coralendeffector.CoralEndEffector;

@Logged
public class RobotContainer {
  private CoralEndEffector endEffector = CoralEndEffector.create();

  private CommandXboxController driver = new CommandXboxController(0);

  public RobotContainer() {
    endEffector.setDefaultCommand(endEffector.runAtVelocity(() -> RPM.zero()));

    configureBindings();
  }

  private void configureBindings() {}

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
