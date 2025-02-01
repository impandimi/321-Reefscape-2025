/* (C) Robolancers 2025 */
package frc.robot.subsystems.algaeIntakeClimb;

import static edu.wpi.first.units.Units.Degrees;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;

public class AlgaeIntakeClimbIOKraken implements AlgaeIntakeClimbIO {
  // kraken implementation of real mechanism
  public static final AlgaeIntakeClimbConfig config = new AlgaeIntakeClimbConfig(0, 0, 0, 0);

  // device ids are plcaeholders
  TalonFX pivotMotorLeft = new TalonFX(1); // corresponds to the left pivot motor
  TalonFX pivotMotorRight = new TalonFX(2); // corresponds to the right pivot motor
  VoltageOut voltageRequest = new VoltageOut(0); // used to set voltage

  private DigitalInput algaeSensor = new DigitalInput(1); // beam break
  private DutyCycleEncoder algaeIntakeClimbEncoder =
      new DutyCycleEncoder(
          1,
          360,
          AlgaeIntakeClimbConstants.kPivotZeroOffsetAngle.in(
              Degrees)); // absolute encoder + configuration stuff

  public AlgaeIntakeClimbIOKraken() {
    pivotMotorLeft // sets up and creates left pivot motor
        .getConfigurator()
        .apply(
            new CurrentLimitsConfigs()
                .withStatorCurrentLimitEnable(true)
                .withStatorCurrentLimit(AlgaeIntakeClimbConstants.kSmartCurrentLimit));
    pivotMotorLeft
        .getConfigurator()
        .apply(
            new MotorOutputConfigs()
                .withNeutralMode(NeutralModeValue.Brake)
                .withInverted(
                    AlgaeIntakeClimbConstants.kPivotInverted
                        ? InvertedValue.CounterClockwise_Positive
                        : InvertedValue.Clockwise_Positive));
    pivotMotorLeft
        .getConfigurator()
        .apply(
            new FeedbackConfigs()
                .withSensorToMechanismRatio(
                    1 / AlgaeIntakeClimbConstants.kPivotPositionConversionFactor));

    pivotMotorRight // same thing with right pivot motor
        .getConfigurator()
        .apply(
            new CurrentLimitsConfigs()
                .withStatorCurrentLimitEnable(true)
                .withStatorCurrentLimit(AlgaeIntakeClimbConstants.kSmartCurrentLimit));
    pivotMotorRight
        .getConfigurator()
        .apply(
            new MotorOutputConfigs()
                .withNeutralMode(NeutralModeValue.Brake)
                .withInverted(
                    AlgaeIntakeClimbConstants.kPivotInverted
                        ? InvertedValue.CounterClockwise_Positive
                        : InvertedValue.Clockwise_Positive));
    pivotMotorRight
        .getConfigurator()
        .apply(
            new FeedbackConfigs()
                .withSensorToMechanismRatio(
                    1 / AlgaeIntakeClimbConstants.kPivotPositionConversionFactor));
  }

  public void setPivotVoltage(Voltage volts) {
    pivotMotorLeft.setControl(
        voltageRequest.withOutput(volts)); // kraken implementation of setvoltage
    pivotMotorRight.setControl(voltageRequest.withOutput(volts));
  }

  public void updateInputs(AlgaeIntakeClimbInputs inputs) { // updates inputs
    inputs.currentPivotAngle = Degrees.of(algaeIntakeClimbEncoder.get());
    inputs.hasAlgae = algaeSensor.get();
  }
}
