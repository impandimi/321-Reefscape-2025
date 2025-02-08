/* (C) Robolancers 2025 */
package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.SuperstructureVisualizer;
import frc.robot.subsystems.algaeIntakePivot.AlgaeIntakePivot;
import frc.robot.subsystems.algaeIntakeRollers.AlgaeIntakeRollers;
import frc.robot.subsystems.coralendeffector.CoralEndEffector;
import frc.robot.subsystems.drivetrain.DrivetrainConstants;
import frc.robot.subsystems.drivetrain.SwerveDrive;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.elevatorarm.ElevatorArm;
import frc.robot.subsystems.elevatorarm.ElevatorArmConstants;

@Logged
public class RobotContainer {

  private SwerveDrive drivetrain = SwerveDrive.create();
  private AlgaeIntakePivot algaePivot = AlgaeIntakePivot.create();
  private AlgaeIntakeRollers algaeRollers = AlgaeIntakeRollers.create();
  private CoralEndEffector coralEndEffector = CoralEndEffector.create();
  private ElevatorArm elevatorArm = ElevatorArm.create();
  private Elevator elevator = Elevator.create();

  private CommandXboxController driver = new CommandXboxController(0);
  private CommandXboxController manipulator = new CommandXboxController(1);

  private SuperstructureVisualizer visualizer =
      new SuperstructureVisualizer(() -> elevator.getHeight(), () -> elevatorArm.getAngle());

  public RobotContainer() {

    drivetrain.setDefaultCommand(
        drivetrain.driveFieldCentric(
            () ->
                -MathUtil.applyDeadband(
                    driver.getLeftY() * DrivetrainConstants.kMaxLinearVelocity.in(MetersPerSecond),
                    DrivetrainConstants.kDriveDeadband),
            () ->
                -MathUtil.applyDeadband(
                    driver.getLeftX() * DrivetrainConstants.kMaxLinearVelocity.in(MetersPerSecond),
                    DrivetrainConstants.kDriveDeadband),
            () ->
                -MathUtil.applyDeadband(
                    driver.getRightX()
                        * DrivetrainConstants.kMaxAngularVelocity.in(RadiansPerSecond),
                    DrivetrainConstants.kRotationDeadband)));

    driver.y().onTrue(elevator.goToHeight(() -> ElevatorConstants.kElevatorMaximumHeight)); // v
    driver.x().onTrue(elevator.goToHeight(() -> ElevatorConstants.kElevatorMinimumHeight)); // c

    driver.b().onTrue(elevatorArm.goToAngle(() -> Degrees.of(90))); // x
    driver.a().onTrue(elevatorArm.goToAngle(() -> ElevatorArmConstants.kMinAngle)); // z

    configureBindings();
  }

  private void configureBindings() {}

  public Command getAutonomousCommand() {
    return Commands.none();
  }
}
