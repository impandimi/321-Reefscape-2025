/* (C) Robolancers 2025 */
package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.AlgaeSuperstructure;
import frc.robot.subsystems.AlgaeSuperstructure.AlgaeSetpoint;
import frc.robot.subsystems.CoralSuperstructure.CoralScorerSetpoint;
import frc.robot.subsystems.algaeIntakePivot.AlgaeIntakePivot;
import frc.robot.subsystems.algaeIntakeRollers.AlgaeIntakeRollers;
import frc.robot.subsystems.drivetrain.DrivetrainConstants;
import frc.robot.util.ReefPosition;
import java.util.function.DoubleSupplier;

@Logged
public class RobotContainer {
  //   private SwerveDrive drivetrain = SwerveDrive.create();
  private AlgaeIntakePivot algaePivot = AlgaeIntakePivot.create();
  private AlgaeIntakeRollers algaeRollers = AlgaeIntakeRollers.create();
  //   private CoralEndEffector coralEndEffector = CoralEndEffector.create();
  //   private ElevatorArm elevatorArm = ElevatorArm.create();
  //   private Elevator elevator = Elevator.create();

  //   private CoralSuperstructure coralSuperstructure =
  //       new CoralSuperstructure(elevator, elevatorArm, coralEndEffector);
  private AlgaeSuperstructure algaeSuperstructure =
      new AlgaeSuperstructure(algaePivot, algaeRollers);

  //   private Vision vision =
  //       Vision.create(
  //           // Java 21 pattern matching switch would be nice
  //           (drivetrain instanceof DrivetrainSim)
  //               ? ((DrivetrainSim) drivetrain)::getActualPose
  //               : drivetrain::getPose,
  //           visionEst ->
  //               drivetrain.addVisionMeasurement(
  //                   visionEst.estimate().estimatedPose.toPose2d(),
  //                   visionEst.estimate().timestampSeconds,
  //                   visionEst.stdDevs()),
  //           reefVisionEst ->
  //               drivetrain.addReefVisionMeasurement(
  //                   reefVisionEst.estimate().estimatedPose.toPose2d(),
  //                   reefVisionEst.estimate().timestampSeconds,
  //                   reefVisionEst.stdDevs()));

  private CommandXboxController driver = new CommandXboxController(0);
  private XboxController manipulator = new XboxController(1);

  // private Trigger isSlowMode = driver.leftBumper();

// private DoubleSupplier driverForward =
//       () ->
//           -MathUtil.applyDeadband(driver.getLeftY(), 0.05)
//               * (isSlowMode.getAsBoolean()
//                   ? 2
//                   : DrivetrainConstants.kMaxLinearVelocity.in(MetersPerSecond));
//   private DoubleSupplier driverStrafe =
//       () ->
//           -MathUtil.applyDeadband(driver.getLeftX(), 0.05)
//               * (isSlowMode.getAsBoolean()
//                   ? 2
//                   : DrivetrainConstants.kMaxLinearVelocity.in(MetersPerSecond));
//   private DoubleSupplier driverTurn = () -> -MathUtil.applyDeadband(driver.getRightX(), 0.05) * 5;

  // robot queued states
  // private ReefPosition queuedReefPosition = ReefPosition.NONE;
  // private CoralScorerSetpoint queuedSetpoint = CoralScorerSetpoint.NEUTRAL;

  //   private SuperstructureVisualizer stateVisualizer =
  //       new SuperstructureVisualizer(
  //           () -> elevator.getHeight(), () -> elevatorArm.getAngle(), () ->
  // algaePivot.getAngle());

  private boolean isDriverOverride = false;

  public RobotContainer() {

    // home everything on robot start
    RobotModeTriggers.disabled()
        .negate()
        .onTrue(algaePivot.homeMechanism().onlyIf(() -> !algaePivot.pivotIsHomed()));
    // .onTrue(HomingCommands.homeEverything(elevator, algaePivot));

    // drive
    // drivetrain.setDefaultCommand(drivetrain.teleopDrive(driverForward, driverStrafe,
    // driverTurn));

    // algae default commands (stalling rollers, default algae pivot setpoint)
    algaeRollers.setDefaultCommand(algaeRollers.setMechanismVoltage(() -> Volts.zero()));
    algaePivot.setDefaultCommand(algaePivot.goToAngle(() -> AlgaeSetpoint.NEUTRAL.getAlgaeAngle()));

    // elevator.setDefaultCommand(
    //     elevator.goToHeight(() -> CoralScorerSetpoint.NEUTRAL.getElevatorHeight()));
    // elevatorArm.setDefaultCommand(
    //     elevatorArm.goToAngle(() -> CoralScorerSetpoint.NEUTRAL.getArmAngle()));
    // coralEndEffector.setDefaultCommand(coralEndEffector.stallCoralIfDetected());

    // when both are about to collide, move elevator out of the way until the algae pivot is out of
    // the collision zone
    // new Trigger(algaePivot::inCollisionZone)
    //     .and(new Trigger(elevator::inCollisionZone))
    //     .onTrue(
    //         elevator
    //             .goToHeight(() -> ElevatorConstants.kElevatorDangerHeight.plus(Meters.of(0.1)))
    //             .until(new Trigger(algaePivot::inCollisionZone).negate()));

    configureBindings();
  }

  private void configureBindings() {
    // driver controls
    // score coral / flip off algae
    driver.y().toggleOnTrue(algaeSuperstructure.prepareClimb());
    driver.a().onTrue(algaeSuperstructure.climb());
    // driver.leftBumper().whileTrue(algaeRollers.intake());
    driver.rightBumper().whileTrue(algaeRollers.outtake());

    // // --- CORAL AUTOMATED CONTROLS ---

    // // coral feeding
    // driver
    //     .rightBumper()
    //     .whileTrue(
    //         StationAlign.rotateToNearestStationTag(drivetrain, driverForward, driverStrafe)
    //             .alongWith(coralSuperstructure.feedCoral()));

    // // coral outtake
    // driver
    //     .rightTrigger()
    //     .whileTrue( // while right trigger is pressed:
    //         Commands.runOnce(() -> isDriverOverride = false)
    //             .andThen(
    //                 // either align to reef or coral based on how far we are away
    //                 // rotate to reef until we're close enough
    //                 ReefAlign.rotateToNearestReefTag(drivetrain, driverForward, driverStrafe)
    //                     .until(
    //                         () ->
    //                             ReefAlign.isWithinReefRange(
    //                                     drivetrain,
    //                                     ReefAlign
    //                                         .kMechanismDeadbandThreshold) // use mechanism
    // threshold
    //                                 // cuz we
    //                                 // wanna be close before aligning
    //                                 // in this case
    //                                 && Math.hypot(
    //                                         driverForward.getAsDouble(),
    // driverStrafe.getAsDouble())
    //                                     <= 0.05
    //                                 && !isDriverOverride)
    //                     .andThen(
    //                         // when we get close enough, align to reef, but only while we're
    // close
    //                         // enough
    //                         ReefAlign.alignToReef(drivetrain, () -> queuedReefPosition)
    //                             .onlyWhile(
    //                                 () ->
    //                                     ReefAlign.isWithinReefRange(
    //                                             drivetrain,
    // ReefAlign.kMechanismDeadbandThreshold)
    //                                         && Math.hypot(
    //                                                 driverForward.getAsDouble(),
    //                                                 driverStrafe.getAsDouble())
    //                                             <= 0.05
    //                                         &&
    //                                         // allow driver control to be taken back when
    //                                         // driverOverride becomes true
    //                                         !isDriverOverride))
    //                     // when we get far away, repeat the command
    //                     .repeatedly()
    //                     .alongWith( // and run the mechanism to where we need to go
    //                         coralSuperstructure
    //                             .goToSetpoint(
    //                                 // move arm up to avoid hitting reef until we get close to
    // reef
    //                                 () -> CoralScorerSetpoint.NEUTRAL.getElevatorHeight(),
    //                                 () -> ElevatorArmConstants.kPreAlignAngle)
    //                             .until(
    //                                 () ->
    //                                     coralSuperstructure.atTargetState()
    //                                         && ReefAlign.isWithinReefRange(
    //                                             drivetrain,
    // ReefAlign.kMechanismDeadbandThreshold))
    //                             .andThen(
    //                                 // move the elevator up but keep arm up
    //                                 coralSuperstructure
    //                                     .goToSetpoint(
    //                                         () -> queuedSetpoint.getElevatorHeight(),
    //                                         () -> ElevatorArmConstants.kPreAlignAngle)
    //                                     .until(() -> coralSuperstructure.atTargetState())
    //                                     // then move arm down to setpoint
    //                                     .andThen(
    //                                         coralSuperstructure.goToSetpoint(() ->
    // queuedSetpoint)))
    //                             // and only do this while we're in the zone (when we're not, we
    // will
    //                             // stay in the pre-alignment position)
    //                             .onlyWhile(
    //                                 () ->
    //                                     ReefAlign.isWithinReefRange(
    //                                             drivetrain,
    // ReefAlign.kMechanismDeadbandThreshold)
    //                                         && queuedSetpoint != CoralScorerSetpoint.NEUTRAL)
    //                             .repeatedly())));

    // driver
    //     .rightTrigger()
    //     .onFalse( // for coral scoring
    //         coralSuperstructure
    //             .goToSetpoint(() -> queuedSetpoint) // ensure we're at the setpoint
    //             .alongWith(coralSuperstructure.outtakeCoral()) // and outtake coral
    //             .until(() -> !coralSuperstructure.hasCoral()) // until we don't have coral
    //             .withTimeout(1) // timeout at 1 second
    //             .andThen(
    //                 // move arm up and go back down (only if we're already at the scoring
    // setpoint
    //                 // state)
    //                 coralSuperstructure
    //                     .goToSetpoint(
    //                         () -> CoralScorerSetpoint.NEUTRAL.getElevatorHeight(),
    //                         () -> ElevatorArmConstants.kPreAlignAngle)
    //                     .until(
    //                         coralSuperstructure::atTargetState)) // and then resume default
    // command
    //             .onlyIf(
    //                 () ->
    //                     coralSuperstructure.atTargetState()
    //                         && queuedSetpoint != CoralScorerSetpoint.NEUTRAL
    //                         && !driver
    //                             .povLeft()
    //                             .getAsBoolean())); // only if we're at the target state and are
    // // ready
    // // to score

    // // --- ALGAE AUTOMATED CONTROLS ---

    // // algae feeding
    driver.leftBumper().whileTrue(algaeSuperstructure.intakeAlgae());}

    // // algae outtake
    // driver
    //     .leftTrigger()
    //     .whileTrue( // while left trigger is pressed:
    //         Commands.runOnce(() -> isDriverOverride = false)
    //             .andThen(
    //                 // rotate to nearest processor unless conditions for full alignment are met
    //                 ProcessorAlign.rotateToNearestProcessor(drivetrain, driverForward,
    // driverStrafe)
    //                     .until(
    //                         () -> // conditions for full alignment: in range + driver not
    // pressing
    //                             // on stick + driver override is off
    //                             ProcessorAlign.isWithinProcessorRange(
    //                                     drivetrain, ProcessorAlign.kAlignmentDeadbandRange)
    //                                 && Math.hypot(
    //                                         driverForward.getAsDouble(),
    // driverStrafe.getAsDouble())
    //                                     <= 0.05
    //                                 && !isDriverOverride)
    //                     .andThen(
    //                         // conditions for full alignment are met, proceed with full alignment
    //                         ProcessorAlign.goToNearestAlign(drivetrain)
    //                             .onlyWhile(
    //                                 () ->
    //                                     !isDriverOverride
    //                                         && Math.hypot(
    //                                                 driverForward.getAsDouble(),
    //                                                 driverStrafe.getAsDouble())
    //                                             <= 0.05
    //                                         && ProcessorAlign.isWithinProcessorRange(
    //                                             drivetrain,
    //                                             ProcessorAlign.kAlignmentDeadbandRange)))
    //                     .repeatedly()
    //                     .alongWith(
    //                         algaeSuperstructure.goToSetpoint(
    //                             AlgaeSetpoint
    //                                 .OUTTAKE)))); // move algae intake to the correct setpoint

    // driver
    //     .leftTrigger()
    //     .onFalse( // when left trigger is let go
    //         algaeSuperstructure
    //             .goToSetpoint(
    //                 AlgaeSetpoint.OUTTAKE) // score until we don't have algae or with 1s timeout
    //             .alongWith(algaeSuperstructure.outtakeAlgae())
    //             .until(() -> !algaeSuperstructure.hasAlgae())
    //             .withTimeout(1)
    //             .onlyIf(
    //                 () ->
    //                     algaeSuperstructure.atTargetState()
    //                         && !driver
    //                             .povLeft()
    //                             .getAsBoolean())); // only if algae intake is at outtake position

    // // toggle driver override
    // driver.povUp().onTrue(Commands.runOnce(() -> isDriverOverride = !isDriverOverride));

    /**
     * Preference 2:
     *
     * <p>Pressing right trigger down all the way performs translation-align/to-setpoint, while
     * pressing it slightly performs the rotation align
     *
     * <p>Driver has override over translation-align/to-setpoint
     */
    // new Trigger(() -> driver.getRightTriggerAxis() >= 0.8)
    //     .whileTrue(
    //         ReefAlign.alignToReef(drivetrain, () -> queuedReefPosition)
    //             .onlyWhile(
    //                 () ->
    //                     ReefAlign.isWithinReefRange(
    //                             drivetrain, ReefAlign.kMaxAlignmentDeadbandThreshold)
    //                         && Math.hypot(driverForward.getAsDouble(),
    // driverStrafe.getAsDouble())
    //                             <= 0.05)
    //             .asProxy()
    //             .repeatedly()
    //             .alongWith(
    //                 coralSuperstructure
    //                     .goToSetpoint(() -> queuedSetpoint)
    //                     .onlyWhile(
    //                         () ->
    //                             ReefAlign.isWithinReefRange(
    //                                 drivetrain, ReefAlign.kMechanismDeadbandThreshold))
    //                     .asProxy()
    //                     .repeatedly()));

    // new Trigger(() -> driver.getRightTriggerAxis() > 0.05 && driver.getRightTriggerAxis() < 0.8)
    //     .whileTrue(ReefAlign.rotateToNearestReefTag(drivetrain, driverForward, driverStrafe));

    // manip controls
    // 1 to 4 - right side L1-L4
    // 5 to 8 - left side L1-L4
    // 9 to 10 - algae low / high

  //   manipTrigger(1)
  //       .onTrue(
  //           Commands.runOnce(
  //               () -> {
  //                 queuedReefPosition = ReefPosition.RIGHT;
  //                 queuedSetpoint = CoralScorerSetpoint.L1;
  //               }));

  //   manipTrigger(2)
  //       .onTrue(
  //           Commands.runOnce(
  //               () -> {
  //                 queuedReefPosition = ReefPosition.RIGHT;
  //                 queuedSetpoint = CoralScorerSetpoint.L2;
  //               }));

  //   manipTrigger(3)
  //       .onTrue(
  //           Commands.runOnce(
  //               () -> {
  //                 queuedReefPosition = ReefPosition.RIGHT;
  //                 queuedSetpoint = CoralScorerSetpoint.L3;
  //               }));

  //   manipTrigger(4)
  //       .onTrue(
  //           Commands.runOnce(
  //               () -> {
  //                 queuedReefPosition = ReefPosition.RIGHT;
  //                 queuedSetpoint = CoralScorerSetpoint.L4;
  //               }));

  //   manipTrigger(5)
  //       .onTrue(
  //           Commands.runOnce(
  //               () -> {
  //                 queuedReefPosition = ReefPosition.LEFT;
  //                 queuedSetpoint = CoralScorerSetpoint.L1;
  //               }));

  //   manipTrigger(6)
  //       .onTrue(
  //           Commands.runOnce(
  //               () -> {
  //                 queuedReefPosition = ReefPosition.LEFT;
  //                 queuedSetpoint = CoralScorerSetpoint.L2;
  //               }));

  //   manipTrigger(7)
  //       .onTrue(
  //           Commands.runOnce(
  //               () -> {
  //                 queuedReefPosition = ReefPosition.LEFT;
  //                 queuedSetpoint = CoralScorerSetpoint.L3;
  //               }));

  //   manipTrigger(8)
  //       .onTrue(
  //           Commands.runOnce(
  //               () -> {
  //                 queuedReefPosition = ReefPosition.LEFT;
  //                 queuedSetpoint = CoralScorerSetpoint.L4;
  //               }));

  //   manipTrigger(9)
  //       .onTrue(
  //           Commands.runOnce(
  //               () -> {
  //                 queuedReefPosition = ReefPosition.ALGAE;
  //                 queuedSetpoint = CoralScorerSetpoint.ALGAE_LOW;
  //               }));

  //   manipTrigger(10)
  //       .onTrue(
  //           Commands.runOnce(
  //               () -> {
  //                 queuedReefPosition = ReefPosition.ALGAE;
  //                 queuedSetpoint = CoralScorerSetpoint.ALGAE_HIGH;
  //               }));
  // }

  private Trigger manipTrigger(int button) {
    return new Trigger(() -> manipulator.getRawButton(button));
  }

  public Command getAutonomousCommand() {
    return Commands.none();
  }
}
