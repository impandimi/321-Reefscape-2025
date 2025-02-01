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
import frc.robot.util.TunableConstant;

// the mechanism that intakes algae low and pivots back to hang from the deep cage
public class AlgaeIntakeClimb extends SubsystemBase {

  private AlgaeIntakeClimbIO io;
  private AlgaeIntakeClimbInputs inputs;

  private PIDController algaeIntakeClimbController; // pid controller, used for pivot
  private ArmFeedforward feedForward; // feed forward for pivot

  private AlgaeIntakeClimbConfig config;

  public AlgaeIntakeClimb(AlgaeIntakeClimbIO io, AlgaeIntakeClimbConfig config) {
    this.io = io;
    this.inputs = new AlgaeIntakeClimbInputs(); // sets io, inputs, and config
    this.config = config;

    algaeIntakeClimbController = new PIDController(config.kP(), config.kI(), config.kD());
    feedForward = new ArmFeedforward(0, config.kG(), 0); // creates pid controller and feed forward
  }

  public Command tune() {
    TunableConstant kP = new TunableConstant("/AlgaeIntakeClimbPivot/kP", config.kP());
    TunableConstant kI = new TunableConstant("/AlgaeIntakeClimbPivot/kI", config.kI());
    TunableConstant kD = new TunableConstant("/AlgaeIntakeClimbPivot/kD", config.kD());
    TunableConstant kG = new TunableConstant("/AlgaeIntakeClimbPivot/kG", config.kG());
    TunableConstant desiredAngle = new TunableConstant("/AlgaeIntakeClimbPivot/desiredAngle", 0);
    // allows us to tune PID and feed forward constants(kP, kI, kD, kG) live on smart dashboard
    // so that we dont have to re run the code every time we change on of them.
    return run(
        () -> {
          this.algaeIntakeClimbController.setPID(kP.get(), kI.get(), kD.get());
          this.feedForward = new ArmFeedforward(0, kG.get(), 0);
          goToAngle(Degrees.of(desiredAngle.get()));
        }); // we set the tuning constants arbitrarily then set a desired angle to go to so that we
    // can see how far off it is and which constants need change
  }

  public static AlgaeIntakeClimb create() {
    return RobotBase.isReal() // TODO: possibly change from spark to kraken
        ? new AlgaeIntakeClimb(
            new AlgaeIntakeClimbIOSpark(),
            AlgaeIntakeClimbIOSpark
                .config) // creates real mechanism if the code is running on a robot
        : new AlgaeIntakeClimb(
            new AlgaeIntakeClimbIOSim(),
            AlgaeIntakeClimbIOSim
                .config); // creates a sim mechanism if the code is not on a real robot
  }

  public static AlgaeIntakeClimb disable() {
    return new AlgaeIntakeClimb(
        new AlgaeIntakeClimbIOIdeal(),
        AlgaeIntakeClimbIOIdeal.config); // creates ideal mechanism for disabled robot
  }

  // get to a desired angle by setting pivot voltage to sum of calculated pid and feedforward
  public void goToAngle(Angle desiredAngle) {
    Voltage desiredVoltage =
        Volts.of(
            feedForward.calculate(desiredAngle.in(Degrees), 0, 0)
                + algaeIntakeClimbController.calculate(
                    inputs.currentPivotAngle.in(Degrees), desiredAngle.in(Degrees)));
    io.setPivotVoltage(desiredVoltage);
  }

  // TODO:be careful if rollers need to start slightly after pivot

  public Command
      outtakePosition() { // position commands only go to the positions for each action, do not
    // execute actions
    return run(() -> goToAngle(AlgaeIntakeClimbConstants.kPivotOuttakeAngle));
  }

  public Command intakePosition() {
    return run(() -> goToAngle(AlgaeIntakeClimbConstants.kPivotIntakeAngle));
  }

  public Command
      climbFloorPosition() { // flips intake to the floor in preparation for climb, which consists
    // of the mechanism pivoting back and clamping onto the cage
    return run(() -> goToAngle(AlgaeIntakeClimbConstants.kPivotFloorAngle));
  }

  public Command climb() { // mechanism clamps onto cage by rotating via pivot
    return run(
        () -> {
          if (inputs.currentPivotAngle.in(Degrees)
              > AlgaeIntakeClimbConstants.kPivotClimbAngle.in(Degrees)) {
            io.setPivotVoltage(AlgaeIntakeClimbConstants.kPivotClimbVoltage);
          } else {
            io.setPivotVoltage(Volts.of(0));
          }
          ;
        });
  }

  public Command setMechanismVoltage(
      Voltage volts) { // sets voltage to the whole mechanism, used for default command
    return run(
        () -> {
          io.setPivotVoltage(volts);
        });
  }

  @Override
  public void periodic() { // updating inputs
    io.updateInputs(inputs);
  }
}
