/* (C) Robolancers 2025 */
package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.drivetrain.DrivetrainConstants;
import frc.robot.subsystems.drivetrain.DrivetrainSim;
import frc.robot.subsystems.drivetrain.SwerveDrive;
import frc.robot.subsystems.drivetrain.TunerConstants;
import frc.robot.util.MathUtils;

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

    drivetrain.setDefaultCommand(
        drivetrain.driveFieldCentric(
            () ->
                -MathUtil.applyDeadband(
                        driverController.getLeftY(), DrivetrainConstants.kDriveDeadband)
                    * DrivetrainConstants.kMaxLinearVelocity,
            () ->
                -MathUtil.applyDeadband(
                        driverController.getLeftX(), DrivetrainConstants.kDriveDeadband)
                    * DrivetrainConstants.kMaxLinearVelocity,
            () ->
                -MathUtils.squareKeepSign(
                        MathUtil.applyDeadband(
                            driverController.getRightX(), DrivetrainConstants.kRotationDeadband))
                    * DrivetrainConstants.kMaxAngularVelocity));

    configureBindings();
  }

  private void configureBindings() {}

  public Command getAutonomousCommand() {
    try {
      return AutoBuilder.followPath(PathPlannerPath.fromPathFile("Example Path"));
    } catch (Exception e) {
      return Commands.none();
    }
  }
}
