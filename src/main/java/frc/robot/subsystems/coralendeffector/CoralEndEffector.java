/* (C) Robolancers 2025 */
package frc.robot.subsystems.coralendeffector;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CoralEndEffector extends SubsystemBase {
  private CoralEndEffectorInputs inputs;
  private CoralEndEffectorIO io;

  public static CoralEndEffector create() {
    return RobotBase.isReal()
        ? new CoralEndEffector(new CoralEndEffectorIOReal())
        : new CoralEndEffector(new CoralEndEffectorIOSim());
  }

  public static CoralEndEffector disable() {
    return new CoralEndEffector(new CoralEndEffectorIOIdeal());
  }

  public CoralEndEffector(CoralEndEffectorIO io) {
    this.io = io;
    this.inputs = new CoralEndEffectorInputs();
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
  }

  public Command intakeCoral() {
    return run(
        () -> {
          io.setVoltage(CoralEndEffectorConstants.kIntakeVoltage);
        });
  }

  public Command outtakeCoral() {
    return run(
        () -> {
          io.setVoltage(CoralEndEffectorConstants.kOuttakeVoltage);
        });
  }
}
