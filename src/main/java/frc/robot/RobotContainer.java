/* (C) Robolancers 2025 */
package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.drivetrain.Drivetrain;

public class RobotContainer {

  Drivetrain drivetrain;
  CommandXboxController driverController = new CommandXboxController(0);

  public RobotContainer() {
    drivetrain = drivetrain.create();

    drivetrain.setDefaultCommand(
        drivetrain.driveFieldCentric(
            driverController.getLeftX(),
            -driverController.getLeftY(),
            driverController.getRightX()));

    configureBindings();
  }

  private void configureBindings() {}

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
