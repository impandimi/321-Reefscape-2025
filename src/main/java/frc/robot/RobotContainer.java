/* (C) Robolancers 2025 */
package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.drivetrain.DrivetrainConstants;
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

    //
    drivetrain.setDefaultCommand(
        drivetrain.teleopDrive(
            () ->
                -MathUtil.applyDeadband(
                        driverController.getLeftY(), DrivetrainConstants.kDriveDeadband)
                    * DrivetrainConstants.kMaxLinearVelocity.in(MetersPerSecond),
            () ->
                -MathUtil.applyDeadband(
                        driverController.getLeftX(), DrivetrainConstants.kDriveDeadband)
                    * DrivetrainConstants.kMaxLinearVelocity.in(MetersPerSecond),
            () ->
                -MathUtil.applyDeadband(
                        driverController.getRightX(), DrivetrainConstants.kRotationDeadband)
                    * DrivetrainConstants.kMaxAngularVelocity.in(RadiansPerSecond)));

    // math for drive fixed heading. Not sure if we need it.
    // Rotation2d.fromRadians(
    //     -Math.PI / 2
    //         + Math.atan2(
    //             MathUtil.applyDeadband(
    //                 -driverController.getRightY(),
    //                 DrivetrainConstants.kRotationDeadband),
    //             MathUtil.applyDeadband(
    //                 driverController.getRightX(),
    //                 DrivetrainConstants.kRotationDeadband)))

    driverController
        .a()
        .whileTrue(
            drivetrain.driveToFieldPose(
                () -> new Pose2d(new Translation2d(2, 2), new Rotation2d())));

    configureBindings();
  }

  private void configureBindings() {}

  public Command getAutonomousCommand() {
    return Commands.none();
  }
}
