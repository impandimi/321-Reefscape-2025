/* (C) Robolancers 2025 */
package frc.robot.subsystems.elevatorarm;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.TunableConstant;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

// NOTE: Convention if we use an arm with two mechanism is: zeroed at CORAL INTAKE on the RIGHT
// TO ZERO: move CORAL all teh way down (use gravity if needed), zero, then move it to 90 deg, then
// zero again
@Logged
public class ElevatorArm extends SubsystemBase {
  private ElevatorArmIO io;
  private ElevatorArmInputs inputs;

  private PIDController pidController;
  private ArmFeedforward feedforward;

  private ElevatorArmConfig config;

  @NotLogged private BooleanSupplier hasCoral = () -> false;
  @NotLogged private BooleanSupplier hasAlgae = () -> false;

  public static ElevatorArm create() {
    return RobotBase.isReal()
        ? new ElevatorArm(new ElevatorArmIOReal(), ElevatorArmIOReal.config)
        : new ElevatorArm(new ElevatorArmIOSim(), ElevatorArmIOSim.config);
  }

  public static ElevatorArm disable() {
    return new ElevatorArm(new ElevatorArmIOIdeal(), ElevatorArmIOIdeal.config);
  }

  public ElevatorArm(ElevatorArmIO io, ElevatorArmConfig config) {
    this.io = io;
    this.config = config;
    this.inputs = new ElevatorArmInputs();
    this.pidController = new PIDController(config.kP(), config.kI(), config.kD());
    this.feedforward = new ArmFeedforward(0, config.kG(), 0);
  }

  private double calculateGamepieceFeedforward() {
    // calculate the amount of feedforward needed to keep a coral and / or an algae
    return (hasCoral.getAsBoolean() ? config.kCoralFF() * Math.cos(inputs.angle.in(Radians)) : 0)
        + (hasAlgae.getAsBoolean()
            ? config.kAlgaeFF() * Math.cos(inputs.angle.plus(Degrees.of(180)).in(Radians))
            : 0);
  }

  public void goToAngle(Angle angle) {
    double volts =
        pidController.calculate(inputs.angle.in(Degrees), angle.in(Degrees))
            + feedforward.calculate(inputs.angle.in(Radians), 0)
            + calculateGamepieceFeedforward();
    System.out.println(volts);
    io.setVoltage(Volts.of(volts));
  }

  public void runVolts(Voltage volts) {
    io.setVoltage(volts);
  }

  public Command goToAngle(Supplier<Angle> angleSup) {
    return run(
        () -> {
          goToAngle(angleSup.get());
        });
  }

  public Command runVolts(Supplier<Voltage> volts) {
    return run(
        () -> {
          runVolts(volts.get());
        });
  }

  public Command tune() {
    TunableConstant kP = new TunableConstant("/ElevatorArm/kP", config.kP());
    TunableConstant kI = new TunableConstant("/ElevatorArm/kI", config.kI());
    TunableConstant kD = new TunableConstant("/ElevatorArm/kD", config.kD());
    TunableConstant kG = new TunableConstant("/ElevatorArm/kG", config.kG());
    TunableConstant kS = new TunableConstant("/ElevatorArm/kS", config.kS());
    TunableConstant targetAngle = new TunableConstant("/ElevatorArm/TargetAngle", 0);

    return run(
        () -> {
          this.pidController.setPID(kP.get(), kI.get(), kD.get());
          this.feedforward = new ArmFeedforward(0, kG.get(), 0);
          goToAngle(Degrees.of(targetAngle.get()));
        });
  }

  public void periodic() {
    io.updateInputs(this.inputs);
  }
}
