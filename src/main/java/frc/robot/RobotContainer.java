/* (C) Robolancers 2025 */
package frc.robot;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.auto.AutomaticAutonomousMaker3000;
import frc.robot.commands.HomingCommands;
import frc.robot.subsystems.AlgaeSuperstructure;
import frc.robot.subsystems.AlgaeSuperstructure.AlgaeSetpoint;
import frc.robot.subsystems.CoralSuperstructure;
import frc.robot.subsystems.CoralSuperstructure.CoralScorerSetpoint;
import frc.robot.subsystems.algaeIntakePivot.AlgaeIntakePivot;
import frc.robot.subsystems.algaeIntakeRollers.AlgaeIntakeRollers;
import frc.robot.subsystems.coralendeffector.CoralEndEffector;
import frc.robot.subsystems.drivetrain.DrivetrainConstants;
import frc.robot.subsystems.drivetrain.SwerveDrive;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.elevatorarm.ElevatorArm;
import frc.robot.util.MathUtils;
import java.util.function.DoubleSupplier;

@Logged
public class RobotContainer {

  private SwerveDrive drivetrain = SwerveDrive.create();
  private AlgaeIntakePivot algaePivot = AlgaeIntakePivot.create();
  private AlgaeIntakeRollers algaeRollers = AlgaeIntakeRollers.create();
  private CoralEndEffector coralEndEffector = CoralEndEffector.create();
  private ElevatorArm elevatorArm = ElevatorArm.create();
  private Elevator elevator = Elevator.create();

  private CoralSuperstructure coralSuperstructure =
      new CoralSuperstructure(elevator, elevatorArm, coralEndEffector);
  private AlgaeSuperstructure algaeSuperstructure =
      new AlgaeSuperstructure(algaePivot, algaeRollers);

      
  private AutomaticAutonomousMaker3000 automaker = new AutomaticAutonomousMaker3000(coralEndEffector, coralSuperstructure);

  private CommandXboxController driver = new CommandXboxController(0);
  private CommandXboxController manipulator = new CommandXboxController(1);

  private Trigger isSlowMode = driver.leftBumper();

  private DoubleSupplier driverForward =
      () ->
          MathUtils.deadband(driver.getLeftY(), 0.05)
              * (isSlowMode.getAsBoolean()
                  ? 2
                  : DrivetrainConstants.kMaxLinearVelocity.in(MetersPerSecond));
  private DoubleSupplier driverStrafe =
      () ->
          MathUtils.deadband(driver.getLeftX(), 0.05)
              * (isSlowMode.getAsBoolean()
                  ? 2
                  : DrivetrainConstants.kMaxLinearVelocity.in(MetersPerSecond));
  private DoubleSupplier driverTurn = () -> MathUtils.deadband(driver.getRightX(), 0.05) * 5;

  private RobotMode mode = RobotMode.CORAL;

  private Trigger isCoralMode = new Trigger(() -> mode == RobotMode.CORAL);
  private Trigger isAlgaeMode = new Trigger(() -> mode == RobotMode.ALGAE);
  private Trigger isClimbMode = new Trigger(() -> mode == RobotMode.CLIMB);

  public RobotContainer() {

    RobotModeTriggers.disabled()
        .negate()
        .onTrue(HomingCommands.homeEverything(elevator, algaePivot));

    drivetrain.setDefaultCommand(drivetrain.teleopDrive(driverForward, driverStrafe, driverTurn));

    algaeRollers.setDefaultCommand(algaeRollers.stallIfHasAlgae());
    algaePivot.setDefaultCommand(algaePivot.goToAngle(() -> AlgaeSetpoint.NEUTRAL.getAlgaeAngle()));

    elevator.setDefaultCommand(
        elevator.goToHeight(() -> CoralScorerSetpoint.NEUTRAL.getElevatorHeight()));
    elevatorArm.setDefaultCommand(
        elevatorArm.goToAngle(() -> CoralScorerSetpoint.NEUTRAL.getArmAngle()));
    coralEndEffector.setDefaultCommand(coralEndEffector.stallCoralIfDetected());

    // when both are about to collide, move elevator out of the way until the algae pivot is out of
    // the collision zone
    new Trigger(algaePivot::inCollisionZone)
        .and(new Trigger(elevator::inCollisionZone))
        .onTrue(
            elevator
                .goToHeight(() -> ElevatorConstants.kElevatorDangerHeight.plus(Meters.of(0.1)))
                // .withInterruptBehavior(InterruptionBehavior.kCancelIncoming)
                .until(new Trigger(algaePivot::inCollisionZone).negate()));

    configureBindings();
  }

  private void configureBindings() {
    // driver controls
    // TODO: Reef Align (RB, LT, RT)
    // TODO: Coral Align (X)

    // manipulator controls
    // switching modes
    // TODO: may need to debounce

    // coral setpoints

    // algae setpoints

    // "action" button; will hopefully do different actions at different setpoints

    // algae intake

    // climb

    // intake coral
  }

  public Command getAutonomousCommand() {
    return automaker.buildAuto(coralEndEffector, coralSuperstructure);
  }

  enum RobotMode {
    CORAL,
    ALGAE,
    CLIMB;
  }
}
