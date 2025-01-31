/* (C) Robolancers 2025 */
package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Amp;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.TunableConstant;
import java.util.function.Supplier;

@Logged
public class Elevator extends SubsystemBase {
  // Initialize all parts of Elevator Susbsytem, as well as pid controller
  private ElevatorIO io;
  private ElevatorInputs inputs;
  private PIDController pidController;
  private ElevatorFeedforward feedForward;
  private ElevatorConfig config;

  // Method that creates the Elevator object as the real/sim io by checking if we're running a sim
  // or not
  public static Elevator create() {
    return RobotBase.isReal()
        ? new Elevator(new ElevatorIOReal(), ElevatorIOReal.config)
        : new Elevator(new ElevatorIOSim(), ElevatorIOSim.config);
  }

  // Eleavtor Constructor
  // Creates Elevator, sets initialized variables to the real/sim values from create() method above
  public Elevator(ElevatorIO io, ElevatorConfig config) {
    this.io = io;
    this.inputs = new ElevatorInputs();
    this.pidController = new PIDController(config.kP(), config.kI(), config.kD());
    this.feedForward = new ElevatorFeedforward(config.kS(), config.kG(), 0);
    io.setEncoderPosition(ElevatorConstants.kElevatorStartingHeight);
  }

  // Below are methods & their commands for simple robot operations
  // Because we need commands for when buttons are pressed, we create methods first then use
  // run(()->{method})

  // Sets voltage
  public void setVoltage(Voltage volts) {
    io.setVoltage(volts);
  }

  public Command setVoltage(Supplier<Voltage> volts) {
    return run(
        () -> {
          setVoltage(volts.get());
        });
  }

  // Goes to height
  public void goToHeight(Distance targetHeight) {
    double motorOutput = pidController.calculate(inputs.height.in(Meters), targetHeight.in(Meters));

    double ff = feedForward.calculate(motorOutput);
    setVoltage(Volts.of(motorOutput + ff));
  }

  public Command goToHeight(Supplier<Distance> targetHeight) {
    return run(
        () -> {
          goToHeight(targetHeight.get());
        });
  }

  // Command to "home" the encoder(go to starting position & set encoder to said position)
  // Sets voltage to -1, makes elevator lower
  // Once current spikes (signaling motor running into resistance) & the V ~0, encoder position is
  // set to 0
  public Command homeEncoder() {
    return setVoltage(() -> Volts.of(-2))
        .until(
            () ->
                (inputs.current.in(Amp) > 25
                    && Math.abs(inputs.velocity.in(MetersPerSecond)) < 0.5))
        .andThen(
            runOnce(
                () -> {
                  io.setEncoderPosition(
                      Meters.of(ElevatorConstants.kElevatorStartingHeight.in(Meters)));
                }));
  }

  // When command is run, tunable constants are created & PID controller values & target height are
  // set to said tunable values
  public Command tune() {
    TunableConstant kP = new TunableConstant("/Elevator/kP", 0);
    TunableConstant kI = new TunableConstant("/Elevator/kI", 0);
    TunableConstant kD = new TunableConstant("/Elevator/kD", 0);
    TunableConstant kG = new TunableConstant("/Elevator/kG", 0);
    TunableConstant kS = new TunableConstant("/Elevator/kS", 0);
    TunableConstant targetHeight = new TunableConstant("/Elevator/targetHeight", 0);

    return goToHeight(
        () -> {
          this.pidController.setPID(kP.get(), kI.get(), kD.get());
          this.feedForward = new ElevatorFeedforward(kS.get(), kG.get(), 0);
          return Meters.of(targetHeight.get());
        });
  }

  // Loops repeatedly
  public void periodic() {
    // Constantly updates inputs
    io.updateInputs(inputs);
  }
}
