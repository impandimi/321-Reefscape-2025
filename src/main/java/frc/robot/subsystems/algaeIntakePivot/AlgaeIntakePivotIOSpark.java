/* (C) Robolancers 2025 */
package frc.robot.subsystems.algaeIntakePivot;

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

public class AlgaeIntakePivotIOSpark implements AlgaeIntakePivotIO {

  public static final AlgaeIntakePivotConfig config = new AlgaeIntakePivotConfig(0, 0, 0, 0);

  // device ids are plcaeholders
  private SparkMax pivotMotorLeft =
      new SparkMax(AlgaeIntakePivotConstants.kPivotMotorLeftIDSpark, MotorType.kBrushless); // corresponds to left and right motors for pivot
  private SparkMax pivotMotorRight = new SparkMax(AlgaeIntakePivotConstants.kPivotMotorRightIDSpark, MotorType.kBrushless);
  private DutyCycleEncoder algaeIntakeClimbEncoder =
      new DutyCycleEncoder(AlgaeIntakePivotConstants.kEncoderID, 360, AlgaeIntakePivotConstants.kPivotZeroOffsetAngle.in(Degrees));

  private DigitalInput algaeSensor = new DigitalInput(AlgaeIntakePivotConstants.kDigitalInputID); // beam break

  public AlgaeIntakePivotIOSpark() {
    configureMotors(); // configures motors once algae spark object
  }

  public void configureMotors() {
    pivotMotorLeft.configure( // configures two spark motors
        new SparkMaxConfig()
            .inverted(AlgaeIntakePivotConstants.kPivotInverted)
            .voltageCompensation(AlgaeIntakePivotConstants.kNominalVoltage.in(Volts))
            .smartCurrentLimit(AlgaeIntakePivotConstants.kSmartCurrentLimit)
            .apply(
                new EncoderConfig()
                    .velocityConversionFactor(
                        AlgaeIntakePivotConstants.kPivotVelocityConversionFactor)
                    .positionConversionFactor(
                        AlgaeIntakePivotConstants.kPivotPositionConversionFactor)),
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    pivotMotorRight.configure(
        new SparkMaxConfig()
            .inverted(AlgaeIntakePivotConstants.kPivotInverted)
            .voltageCompensation(AlgaeIntakePivotConstants.kNominalVoltage.in(Volts))
            .smartCurrentLimit(AlgaeIntakePivotConstants.kSmartCurrentLimit)
            .follow(AlgaeIntakePivotConstants.kPivotMotorLeftIDSpark),
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
  }

  public void setPivotVoltage(Voltage volts) {
    pivotMotorLeft.setVoltage(volts);
  }

  public void updateInputs(AlgaeIntakePivotInputs inputs) {
    inputs.currentPivotAngle = Degrees.of(algaeIntakeClimbEncoder.get());
    inputs.hasAlgae = algaeSensor.get();
  }
}
