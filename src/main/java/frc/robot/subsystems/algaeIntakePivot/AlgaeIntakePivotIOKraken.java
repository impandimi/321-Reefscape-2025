/* (C) Robolancers 2025 */
package frc.robot.subsystems.algaeIntakePivot;

import static edu.wpi.first.units.Units.Degrees;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DutyCycleEncoder;

@Logged
public class AlgaeIntakePivotIOKraken implements AlgaeIntakePivotIO {
  // kraken implementation of real mechanism
  public static final AlgaeIntakePivotConfig config = new AlgaeIntakePivotConfig(0, 0, 0, 0);

  TalonFX pivotMotorLeft =
      new TalonFX(
          AlgaeIntakePivotConstants.kPivotMotorLeftId); // corresponds to the left pivot motor
  TalonFX pivotMotorRight =
      new TalonFX(
          AlgaeIntakePivotConstants.kPivotMotorRightId); // corresponds to the right pivot motor

  VoltageOut voltageRequest = new VoltageOut(0); // used to set voltage
  Follower followRequest =
      new Follower(
          AlgaeIntakePivotConstants.kPivotMotorLeftId, AlgaeIntakePivotConstants.kRightInverted);

  // absolute encoder + configuration stuff
  private DutyCycleEncoder algaeIntakeClimbEncoder =
      new DutyCycleEncoder(
          AlgaeIntakePivotConstants.kEncoderId,
          360,
          AlgaeIntakePivotConstants.kPivotZeroOffsetAngle.in(Degrees));

  public AlgaeIntakePivotIOKraken() {
    pivotMotorLeft // sets up and creates left pivot motor
        .getConfigurator()
        .apply(
            new CurrentLimitsConfigs()
                .withStatorCurrentLimitEnable(true)
                .withStatorCurrentLimit(AlgaeIntakePivotConstants.kSmartCurrentLimit));
    pivotMotorLeft
        .getConfigurator()
        .apply(
            new MotorOutputConfigs()
                .withNeutralMode(NeutralModeValue.Brake)
                .withInverted(
                    AlgaeIntakePivotConstants.kLeftInverted
                        ? InvertedValue.CounterClockwise_Positive
                        : InvertedValue.Clockwise_Positive));
    pivotMotorLeft
        .getConfigurator()
        .apply(
            new FeedbackConfigs()
                .withSensorToMechanismRatio(
                    1 / AlgaeIntakePivotConstants.kPivotPositionConversionFactor));

    pivotMotorRight // same thing with right pivot motor
        .getConfigurator()
        .apply(
            new CurrentLimitsConfigs()
                .withStatorCurrentLimitEnable(true)
                .withStatorCurrentLimit(AlgaeIntakePivotConstants.kSmartCurrentLimit));
    pivotMotorRight
        .getConfigurator()
        .apply(
            new MotorOutputConfigs()
                .withNeutralMode(NeutralModeValue.Brake)
                .withInverted(
                    AlgaeIntakePivotConstants.kRightInverted
                        ? InvertedValue.CounterClockwise_Positive
                        : InvertedValue.Clockwise_Positive));
    pivotMotorRight
        .getConfigurator()
        .apply(
            new FeedbackConfigs()
                .withSensorToMechanismRatio(AlgaeIntakePivotConstants.kPivotGearing));
  }

  public void setPivotVoltage(Voltage volts) {
    pivotMotorLeft.setControl(
        voltageRequest.withOutput(volts)); // kraken implementation of setvoltage
    pivotMotorRight.setControl(followRequest);
  }

  public void updateInputs(AlgaeIntakePivotInputs inputs) { // updates inputs
    inputs.pivotAngle = Degrees.of(algaeIntakeClimbEncoder.get());
    inputs.pivotVelocity = pivotMotorLeft.getVelocity().getValue(); 
  }
}
