/* (C) Robolancers 2025 */
package frc.robot.subsystems.algaeIntakeClimb;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

// subsystem

public class AlgaeIntakeClimb extends SubsystemBase {

  private AlgaeIntakeClimbIO io;
  private AlgaeIntakeClimbInputs inputs;
  // intance variables

  private PIDController algaeIntakeClimbController;
  private ArmFeedforward feedForward;

  public AlgaeIntakeClimb(AlgaeIntakeClimbIO io, AlgaeIntakeClimbConfig config) {
    this.io = io;
    this.inputs = new AlgaeIntakeClimbInputs();

    algaeIntakeClimbController = new PIDController(config.kP(), config.kI(), config.kD());
    feedForward = new ArmFeedforward(0, config.kG(), 0);
  }

  public static AlgaeIntakeClimb create() {
    return RobotBase.isReal()
        ? new AlgaeIntakeClimb(new AlgaeIntakeClimbIOSpark(), AlgaeIntakeClimbIOSpark.config)
        : new AlgaeIntakeClimb(new AlgaeIntakeClimbIOSim(), AlgaeIntakeClimbIOSim.config);
  }

  public static AlgaeIntakeClimb disable() {
    return new AlgaeIntakeClimb(new AlgaeIntakeClimbIOIdeal(), AlgaeIntakeClimbIOIdeal.config);
  }

  // goes to angle
  public void goToAngle(Angle desiredAngle) {
    io.setPivotVoltage(
        Volts.of(
            feedForward.calculate(desiredAngle.in(Degrees), 0, 0)
                + algaeIntakeClimbController.calculate(
                    inputs.currentPivotAngle.in(Degrees), desiredAngle.in(Degrees))));
  }

  public void spinRollers(Voltage volts) {
    io.setRollerVoltage(volts);
  }

  // TODO:be careful if rollers need to start slightly after pivot

  public Command outtakePosition() {
    return run(() -> goToAngle(AlgaeIntakeClimbConstants.kPivotOuttakeAngle));
  }

  public Command outtake() {
    return run(() -> spinRollers(AlgaeIntakeClimbConstants.kRollerOuttakePower))
        .until(() -> !inputs.hasAlgae);
  }

  public Command intake() {
    return run(() -> spinRollers(AlgaeIntakeClimbConstants.kRollerIntakePower))
        .alongWith(run(() -> goToAngle(AlgaeIntakeClimbConstants.kIntakeAngle)))
        .until(() -> inputs.hasAlgae);
  }

  public Command climbFloorPosition() { // to get into climbing position
    return run(() -> goToAngle(AlgaeIntakeClimbConstants.kPivotFloorAngle));
  }

  public Command climb() {
    return run(
        () -> {
          if (inputs.currentPivotAngle.in(Degrees)
              > AlgaeIntakeClimbConstants.kPivotClimbAngle.in(Degrees)) {
            io.setPivotVoltage(AlgaeIntakeClimbConstants.kPivotClimbPower);
          } else {
            io.setPivotVoltage(Volts.of(0));
          }
          ;
        });
  }

  public Command unclimb() {
    return run(() -> io.setPivotVoltage(AlgaeIntakeClimbConstants.kPivotUnclimbPower));
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
  }
}
