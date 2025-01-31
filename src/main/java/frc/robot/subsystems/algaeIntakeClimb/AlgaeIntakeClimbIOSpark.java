/* (C) Robolancers 2025 */
package frc.robot.subsystems.algaeIntakeClimb;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;

public class AlgaeIntakeClimbIOSpark implements AlgaeIntakeClimbIO {

  public static final AlgaeIntakeClimbConfig config = new AlgaeIntakeClimbConfig(0, 0, 0, 0);

  // device ids are plcaeholders
  private SparkMax pivotMotorLeft = new SparkMax(1, MotorType.kBrushless);
  private SparkMax pivotMotorRight = new SparkMax(3, MotorType.kBrushless);
  AbsoluteEncoder algaeIntakeClimbEncoder = pivotMotorLeft.getAbsoluteEncoder();

  DigitalInput algaeSensor = new DigitalInput(1);

  private SparkMax rollerMotorLeft = new SparkMax(3, MotorType.kBrushless);

  public AlgaeIntakeClimbIOSpark() {
    configureMotors();
  }

  public void configureMotors() {
    pivotMotorLeft.configure(
        new SparkMaxConfig()
            .inverted(AlgaeIntakeClimbConstants.kInverted)
            .voltageCompensation(AlgaeIntakeClimbConstants.kNominalVoltage.in(Volts))
            .smartCurrentLimit(AlgaeIntakeClimbConstants.kSmartCurrentLimit)
            .apply(
                new EncoderConfig()
                    .velocityConversionFactor(AlgaeIntakeClimbConstants.kVelocityConversionFactor)
                    .positionConversionFactor(AlgaeIntakeClimbConstants.kPositionConversionFactor)),
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    pivotMotorRight.configure(
        new SparkMaxConfig()
            .inverted(AlgaeIntakeClimbConstants.kInverted)
            .voltageCompensation(AlgaeIntakeClimbConstants.kNominalVoltage.in(Volts))
            .smartCurrentLimit(AlgaeIntakeClimbConstants.kSmartCurrentLimit),
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    rollerMotorLeft.configure(
        new SparkMaxConfig()
            .inverted(AlgaeIntakeClimbConstants.kInverted)
            .voltageCompensation(AlgaeIntakeClimbConstants.kNominalVoltage.in(Volts))
            .smartCurrentLimit(AlgaeIntakeClimbConstants.kSmartCurrentLimit),
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
  }

  public void setPivotVoltage(Voltage volts) {
    pivotMotorLeft.setVoltage(volts);
    pivotMotorRight.setVoltage(volts);
  }

  public void setRollerVoltage(Voltage volts) {
    rollerMotorLeft.setVoltage(volts);
  }

  public void updateInputs(AlgaeIntakeClimbInputs inputs) {
    inputs.pivotVelocity = RPM.of(algaeIntakeClimbEncoder.getVelocity());
    inputs.currentPivotAngle = Degrees.of(algaeIntakeClimbEncoder.getPosition());
    inputs.hasAlgae = algaeSensor.get();
    inputs.rollerVelocity = RPM.of(rollerMotorLeft.getEncoder().getVelocity());
  }
}
