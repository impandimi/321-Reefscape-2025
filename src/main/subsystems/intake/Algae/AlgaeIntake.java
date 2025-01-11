/* (C) Robolancers 2025 */
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AlgaeIntake extends SubsystemBase {

  private final AlgaeIntakeIO io;
  private final AlgaeIntakeInputs inputs;

  public static AlgaeIntake create() {
    return RobotBase.isReal()
        ? new AlgaeIntake(new AlgaeIntakeIOReal())
        : new AlgaeIntake(new AlgaeIntakeIOSim());
  }

  public static AlgaeIntake disable() {
    return new AlgaeIntake(new AlgaeIntakeIOIdeal());
  }

  private Intake(IntakeIo io) {
    this.io = io;
    this.inputs = new IntakeInputs();
  }

  @Override
  public void periodic() {
    this.io.updateInputs();
  }

  public Command intakeAlgae() {
    return runVoltage(() -> AlgaeIntakeConstants.kIntakeVoltage);
  }

  public Command outtakeAlgae() {
    return runVoltage(() -> -AlgaeIntakeConstants.kIntakeVoltage);
  }
}
