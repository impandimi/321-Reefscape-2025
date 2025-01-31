/* (C) Robolancers 2025 */
package frc.robot.subsystems.elevatorarm;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DutyCycleEncoder;

/**
 * Implementation of the ElevatorArmIO that controls a real ElevatorArm using a TalonFX Motor
 * Controller
 */
@Logged
public class ElevatorArmIOTalon implements ElevatorArmIO {

  // tuning config for the ElevatorArmIOReal
  public static final ElevatorArmConfig config = new ElevatorArmConfig(0, 0, 0, 0, 0, 0);

  // the motor that is controlling the arm (using a TalonFX controller)
  private TalonFX armMotor = new TalonFX(ElevatorArmConstants.kElevatorArmId);

  // absolute encoder from 0 to 360
  private DutyCycleEncoder encoder =
      new DutyCycleEncoder(
          ElevatorArmConstants.kAbsoluteEncoderPort,
          360,
          ElevatorArmConstants.kAbsoluteEncoderOffset.in(Degrees));

  // TODO: if elec uses a CANdi, use this for encoder output
  // private CANdi candi = new CANdi(0);

  // request to control the arm motor using voltage
  private VoltageOut voltageRequest = new VoltageOut(0);

  public ElevatorArmIOTalon() {
    // setup arm motor
    armMotor
        .getConfigurator()
        .apply(
            new CurrentLimitsConfigs()
                .withStatorCurrentLimitEnable(true)
                .withStatorCurrentLimit(ElevatorArmConstants.kCurrentLimit));
    armMotor
        .getConfigurator()
        .apply(
            new MotorOutputConfigs()
                .withNeutralMode(NeutralModeValue.Brake)
                .withInverted(
                    ElevatorArmConstants.kInverted
                        ? InvertedValue.CounterClockwise_Positive
                        : InvertedValue.Clockwise_Positive));
    armMotor
        .getConfigurator()
        .apply(
            new FeedbackConfigs()
                .withSensorToMechanismRatio(ElevatorArmConstants.kPositionConversionFactor));
  }

  // update inputs from the arm motor
  public void updateInputs(ElevatorArmInputs inputs) {
    inputs.angle = Degrees.of(encoder.get());
    // TODO: see line 37
    // inputs.angle =
    // candi.getPWM1Position(true).getValue().plus(ElevatorArmConstants.kAbsoluteEncoderOffset);
    inputs.velocity = armMotor.getVelocity().getValue();
    inputs.current = armMotor.getTorqueCurrent().getValue();
  }

  // set voltage to the arm motor
  public void setVoltage(Voltage volts) {
    double voltsWithStall = volts.in(Volts);
    // TODO: uncomment if arm gearbox exploding becomes an actual issue
    // if (armMotor.getOutputCurrent() > 40) voltsWithStall /= 60; // jank way to make the motor
    // essentially stop
    armMotor.setControl(voltageRequest.withOutput(voltsWithStall));
  }
}
