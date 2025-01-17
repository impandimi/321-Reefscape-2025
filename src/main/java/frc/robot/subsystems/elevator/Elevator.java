/* (C) Robolancers 2025 */
package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.elevatorarm.ElevatorArmConfig;
import frc.robot.util.TunableConstant;
import java.util.function.Supplier;

public class Elevator extends SubsystemBase {
  private ElevatorIO io;
  private ElevatorInputs inputs;
  private PIDController pidController;
  private ElevatorFeedforward feedForward;
  private ElevatorArmConfig config;

  public static Elevator create() {
    return RobotBase.isReal()
        ? new Elevator(new ElevatorIOReal(), ElevatorIOReal.config)
        : new Elevator(new ElevatorIOSim(), ElevatorIOSim.config);
  }

  public Elevator(ElevatorIO io, ElevatorConfig config) {
    this.io = io;
    this.inputs = new ElevatorInputs();
    this.pidController = new PIDController(config.kP(), config.kI(), config.kD());
    this.feedForward = new ElevatorFeedforward(0, config.kG(), 0);
  }

  // Set voltage
  public void setVoltage(Voltage volts) {
    io.setVoltage(volts);
  }

  public Command setVoltage(Supplier<Voltage> volts) {
    return run(
        () -> {
          setVoltage(volts.get());
        });
  }

  // Go to height
  public void goToHeight(Distance targetHeight) {
    double ff = feedForward.calculate(0.0);
    double motorOutput = pidController.calculate(inputs.height.in(Meters), targetHeight.in(Meters));
    setVoltage(Volts.of(motorOutput + ff));
  }

  public Command goToHeight(Supplier<Distance> targetHeight) {
    return run(
        () -> {
          goToHeight(targetHeight.get());
        });
  }

  // TODO: Command to uh... set encoder pos (height) to 0 when current spikes
  // 1st: set voltage to -1 or something
  // 2nd: do the above UNTIL current reaches a high of something AND maybe velocity ~= 0
  // 3rd: set encoder to zero
  //

  public Command tune() {
    TunableConstant kP = new TunableConstant("/Elevator/kP", 0);
    TunableConstant kI = new TunableConstant("/Elevator/kI", 0);
    TunableConstant kD = new TunableConstant("/Elevator/kD", 0);
    TunableConstant kG = new TunableConstant("/Elevator/kG", 0);
    TunableConstant targetHeight = new TunableConstant("/Elevator/targetHeight", 0);

    return run(
        () -> {
          this.pidController.setPID(kP.get(), kI.get(), kD.get());
          this.feedForward = new ElevatorFeedforward(0, kG.get(), 0);
          goToHeight(Meters.of(targetHeight.get()));
        });
  }

  // Tune :(

  public void periodic() {
    io.updateInputs(inputs);
  }
}
