/* (C) Robolancers 2025 */
package frc.robot.subsystems.algaeIntakeClimb;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;

public class AlgaeIntakeClimbIOKraken implements AlgaeIntakeClimbIO {

  public static final AlgaeIntakeClimbConfig config = new AlgaeIntakeClimbConfig(0, 0, 0, 0);

  // device ids are plcaeholders
  TalonFX pivotMotorLeft = new TalonFX(1);
  TalonFX pivotMotorRight = new TalonFX(2);
  VoltageOut voltageRequest = new VoltageOut(0);

  DigitalInput algaeSensor = new DigitalInput(1);

  private SparkMax rollerMotorLeft = new SparkMax(3, MotorType.kBrushless);

  public AlgaeIntakeClimbIOKraken() {
    configureMotors();
    pivotMotorLeft
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
                    AlgaeIntakeClimbConstants.kInverted
                        ? InvertedValue.CounterClockwise_Positive
                        : InvertedValue.Clockwise_Positive));
    pivotMotorLeft
        .getConfigurator()
        .apply(
            new FeedbackConfigs()
                .withSensorToMechanismRatio(
                    1 / AlgaeIntakeClimbConstants.kPositionConversionFactor));
  }

  public void configureMotors() {
    rollerMotorLeft.configure(
        new SparkMaxConfig()
            .inverted(AlgaeIntakeClimbConstants.kInverted)
            .voltageCompensation(AlgaeIntakeClimbConstants.kNominalVoltage.in(Volts))
            .smartCurrentLimit(AlgaeIntakeClimbConstants.kSmartCurrentLimit),
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
  }

  public void setPivotVoltage(Voltage volts) {
    pivotMotorLeft.setControl(voltageRequest.withOutput(volts));
    pivotMotorRight.setControl(voltageRequest.withOutput(volts));
  }

  public void setRollerVoltage(Voltage volts) {
    rollerMotorLeft.setVoltage(volts);
  }

  public void updateInputs(AlgaeIntakeClimbInputs inputs) {
    inputs.pivotVelocity = pivotMotorLeft.getVelocity().getValue();
    inputs.currentPivotAngle = pivotMotorLeft.getPosition().getValue();
    inputs.hasAlgae = algaeSensor.get();
    inputs.rollerVelocity = RPM.of(rollerMotorLeft.getEncoder().getVelocity());
  }
}
