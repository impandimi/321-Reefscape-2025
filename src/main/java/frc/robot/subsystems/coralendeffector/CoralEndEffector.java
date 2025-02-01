/* (C) Robolancers 2025 */
package frc.robot.subsystems.coralendeffector;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.Supplier;

// coral end effector subsystem
@Logged
public class CoralEndEffector extends SubsystemBase {
  private CoralEndEffectorInputs inputs;
  private CoralEndEffectorIO io;

  private PIDController endEffectorController;
  private SimpleMotorFeedforward feedforward;

  public static CoralEndEffector create() {
    return RobotBase.isReal()
        ? new CoralEndEffector(new CoralEndEffectorIOReal(), CoralEndEffectorIOReal.config)
        : new CoralEndEffector(new CoralEndEffectorIOSim(), CoralEndEffectorIOSim.config);
  }

  public static CoralEndEffector disable() {
    return new CoralEndEffector(new CoralEndEffectorIOIdeal(), CoralEndEffectorIOIdeal.config);
  }

  public CoralEndEffector(CoralEndEffectorIO io, CoralEndEffectorConfig config) {
    this.io = io;
    this.inputs = new CoralEndEffectorInputs();
    this.endEffectorController = new PIDController(config.kP(), config.kI(), config.kD());
    this.feedforward = new SimpleMotorFeedforward(0, config.kV());
  }

  // constantly updates inputs
  @Override
  public void periodic() {
    io.updateInputs(inputs);
  }

  public Command runAtVelocity(Supplier<AngularVelocity> velocity) {
    return run(
        () -> {
          double output =
              endEffectorController.calculate(inputs.velocity.in(RPM), velocity.get().in(RPM))
                  + feedforward.calculate(velocity.get().in(RPM));
          io.setVoltage(Volts.of(output));
        });
  }

  // intakes coral
  public Command intakeCoral() {
    return run(
        () -> {
          io.setVoltage(CoralEndEffectorConstants.kIntakeVoltage);
        });
  }

  // outtakes coral
  public Command outtakeCoral() {
    return run(
        () -> {
          io.setVoltage(CoralEndEffectorConstants.kOuttakeVoltage);
        });
  }

  // stalls coral if we have a coral; this should be the default command
  public Command stallCoralIfDetected() {
    return run(
        () -> {
          if (inputs.hasCoral) {
            io.setVoltage(CoralEndEffectorConstants.kStallVoltage);
          } else {
            io.setVoltage(Volts.zero());
          }
        });
  }

  public Command tune() {
    // TODO: implement
    return Commands.none();
  }
}
