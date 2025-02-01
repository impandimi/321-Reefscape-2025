/* (C) Robolancers 2025 */
package frc.robot.subsystems.algaeIntakeRollers;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.TunableConstant;

// the same mechanism as algaeIntakeClimb but this controls the rollers instead of the pivot
import edu.wpi.first.epilogue.Logged;
@Logged
public class AlgaeIntakeRollers extends SubsystemBase {

  private AlgaeIntakeRollersIO io;
  private AlgaeIntakeRollersInputs inputs;

  private PIDController rollerController;
  private SimpleMotorFeedforward feedForward;

  private AlgaeIntakeRollersConfig config;

  public AlgaeIntakeRollers(AlgaeIntakeRollersIO io, AlgaeIntakeRollersConfig config) {
    this.io = io;
    this.inputs = new AlgaeIntakeRollersInputs();
    this.config = config;

    rollerController = new PIDController(config.kP(), config.kI(), config.kD());
    feedForward = new SimpleMotorFeedforward(0,config.kV());
  }

  public static AlgaeIntakeRollers create() {
    return RobotBase.isReal()
        ? new AlgaeIntakeRollers(
            new AlgaeIntakeRollersIOSpark(), AlgaeIntakeRollersIOSpark.config) // creates real mechanism if robot, sim if no robot,
        // ideal if disabled robot
        : new AlgaeIntakeRollers(new AlgaeIntakeRollersIOSim(), AlgaeIntakeRollersIOSim.config);
  }

  public static AlgaeIntakeRollers disable() {
    return new AlgaeIntakeRollers(new AlgaeIntakeRollersIOIdeal(), AlgaeIntakeRollersIOIdeal.config);
  }

 public Command tune() {
    TunableConstant kP = new TunableConstant("/AlgaeIntakeClimbRollers/kP", config.kP());
    TunableConstant kI = new TunableConstant("/AlgaeIntakeClimbRollers/kI", config.kI());
    TunableConstant kD = new TunableConstant("/AlgaeIntakeClimbRollers/kD", config.kD());
    TunableConstant kV = new TunableConstant("/AlgaeIntakeClimbRollers/kG", config.kV());
    TunableConstant desiredAngularVelocity = new TunableConstant("/AlgaeIntakeClimbRollers/desiredAngular", 0);
    // allows us to tune PID and feed forward constants(kP, kI, kD, kG) live on smart dashboard
    // so that we dont have to re run the code every time we change on of them.
    return run(
        () -> {
          this.rollerController.setPID(kP.get(), kI.get(), kD.get());
          this.feedForward = new SimpleMotorFeedforward(0,kV.get());
          goToAngularVelocity(RPM.of(desiredAngularVelocity.get()));
        }); // we set the tuning constants arbitrarily then set a desired angle to go to so that we
    // can see how far off it is and which constants need change
  }

  public void spinRollers(Voltage volts) {
    io.setRollerVoltage(volts);
  }

  public void goToAngularVelocity(AngularVelocity desiredAngularVelocity) {
    io.setRollerVoltage(
        Volts.of(
            rollerController.calculate(inputs.rollerVelocity.in(RPM), desiredAngularVelocity.in(RPM)) 
            + feedForward.calculate(desiredAngularVelocity.in(RPM))));
  }

  public Command
      intake() { // intakes algae until beam break breaks and registers algae in the mechanism
    return run(() -> spinRollers(AlgaeIntakeRollersConstants.kRollerIntakeVoltage));
  }

  public Command outtake() { // outtakes by spinning rollers outward
    return run(() -> spinRollers(AlgaeIntakeRollersConstants.kRollerOuttakeVoltage));
  }

  public Command setMechanismVoltage(Voltage volts) { // sets whole mechanism voltage
    return run(
        () -> spinRollers(volts)
        );
  }

  @Override // updates inputs constatly
  public void periodic() {
    io.updateInputs(inputs);
  }
}
