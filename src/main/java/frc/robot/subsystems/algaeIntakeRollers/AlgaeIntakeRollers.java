/* (C) Robolancers 2025 */
package frc.robot.subsystems.algaeIntakeRollers;

import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

// the same mechanism as algaeIntakeClimb but this controls the rollers instead of the pivot
public class AlgaeIntakeRollers extends SubsystemBase {

  private AlgaeIntakeRollersIO io;
  private AlgaeIntakeRollersInputs inputs;

  public AlgaeIntakeRollers(AlgaeIntakeRollersIO io) {
    this.io = io;
    this.inputs = new AlgaeIntakeRollersInputs();
  }

  public static AlgaeIntakeRollers create() {
    return RobotBase.isReal() // TODO: possibly change from spark to kraken
        ? new AlgaeIntakeRollers(
            new AlgaeIntakeRollersIOSpark()) // creates real mechanism if robot, sim if no robot,
        // ideal if disabled robot
        : new AlgaeIntakeRollers(new AlgaeIntakeRollersIOSim());
  }

  public static AlgaeIntakeRollers disable() {
    return new AlgaeIntakeRollers(new AlgaeIntakeRollersIOIdeal());
  }

  public void spinRollers(Voltage volts) {
    io.setRollerVoltage(volts);
  }

  public Command
      intake() { // intakes algae until beam break breaks and registers algae in the mechanism
    return run(() -> spinRollers(AlgaeIntakeRollersConstants.kRollerIntakeVoltage))
        .until(() -> inputs.hasAlgae);
  }

  public Command intakeForever() { // intakes without end condition for testing or flexibility
    return run(() -> spinRollers(AlgaeIntakeRollersConstants.kRollerIntakeVoltage));
  }

  public Command outtake() { // outtakes by spinning rollers outward
    return run(() -> spinRollers(AlgaeIntakeRollersConstants.kRollerOuttakeVoltage));
  }

  public Command setMechanismVoltage(Voltage volts) { // sets whole mechanism voltage
    return run(
        () -> {
          io.setRollerVoltage(volts);
        });
  }

  @Override // updates inputs constatly
  public void periodic() {
    io.updateInputs(inputs);
  }
}
