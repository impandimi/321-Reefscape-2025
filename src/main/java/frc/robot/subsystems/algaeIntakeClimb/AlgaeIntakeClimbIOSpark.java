/* (C) Robolancers 2025 */
package frc.robot.subsystems.algaeIntakeClimb;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;

// spark implementation of real mechanism

public class AlgaeIntakeClimbIOSpark implements AlgaeIntakeClimbIO {

  public static final AlgaeIntakeClimbConfig config = new AlgaeIntakeClimbConfig(0, 0, 0, 0);

  // device ids are plcaeholders
  private SparkMax pivotMotorLeft =
      new SparkMax(1, MotorType.kBrushless); // corresponds to left and right motors for pivot
  private SparkMax pivotMotorRight = new SparkMax(3, MotorType.kBrushless);
  private DutyCycleEncoder algaeIntakeClimbEncoder =
      new DutyCycleEncoder(1, 360, AlgaeIntakeClimbConstants.kPivotZeroOffsetAngle.in(Degrees));

  private DigitalInput algaeSensor = new DigitalInput(1); // beam break

  public AlgaeIntakeClimbIOSpark() {
    configureMotors(); // configures motors once algae spark object
  }

  public void configureMotors() {
    pivotMotorLeft.configure( // configures two spark motors
        new SparkMaxConfig()
            .inverted(AlgaeIntakeClimbConstants.kPivotInverted)
            .voltageCompensation(AlgaeIntakeClimbConstants.kNominalVoltage.in(Volts))
            .smartCurrentLimit(AlgaeIntakeClimbConstants.kSmartCurrentLimit)
            .apply(
                new EncoderConfig()
                    .velocityConversionFactor(
                        AlgaeIntakeClimbConstants.kPivotVelocityConversionFactor)
                    .positionConversionFactor(
                        AlgaeIntakeClimbConstants.kPivotPositionConversionFactor)),
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    pivotMotorRight.configure(
        new SparkMaxConfig()
            .inverted(AlgaeIntakeClimbConstants.kPivotInverted)
            .voltageCompensation(AlgaeIntakeClimbConstants.kNominalVoltage.in(Volts))
            .smartCurrentLimit(AlgaeIntakeClimbConstants.kSmartCurrentLimit),
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
  }

  public void setPivotVoltage(Voltage volts) {
    pivotMotorLeft.setVoltage(volts);
    pivotMotorRight.setVoltage(volts);
  }

  public void updateInputs(AlgaeIntakeClimbInputs inputs) {
    inputs.currentPivotAngle = Degrees.of(algaeIntakeClimbEncoder.get());
    inputs.hasAlgae = algaeSensor.get();
  }
}
