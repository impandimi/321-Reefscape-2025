/* (C) Robolancers 2025 */
package frc.robot;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.drivetrain.DrivetrainSim;
import frc.robot.subsystems.drivetrain.SwerveDrive;
import frc.robot.subsystems.drivetrain.TunerConstants;

@Logged
public class RobotContainer {

  SwerveDrive drivetrain;
  CommandXboxController driverController = new CommandXboxController(0);

  public RobotContainer() {
    drivetrain =
        RobotBase.isReal()
            ? new Drivetrain(
                TunerConstants.kTunerDrivetrain.getDriveTrainConstants(),
                TunerConstants.kTunerDrivetrain.getModuleConstants())
            : new DrivetrainSim();

    configureBindings();
  }

  private void configureBindings() {}

  public Command getAutonomousCommand() {
    return Commands.none();
  }
}
