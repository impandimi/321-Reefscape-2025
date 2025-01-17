/* (C) Robolancers 2025 */
package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.units.measure.Voltage;

public class ElevatorIOReal implements ElevatorIO {

  public static final ElevatorConfig config = new ElevatorConfig(0, 0, 0, 0, 0);

  public SparkMax elevatorMotorLeft =
      new SparkMax(ElevatorConstants.kLeftMotorID, MotorType.kBrushless);
  public SparkMax elevatorMotorRight =
      new SparkMax(ElevatorConstants.kRightMotorID, MotorType.kBrushless);

  public ElevatorIOReal() {
    setupMotors();
  }

  public void updateInputs(ElevatorInputs inputs) {
    inputs.height = Meters.of(elevatorMotorLeft.getEncoder().getPosition());
    inputs.velocity = MetersPerSecond.of(elevatorMotorLeft.getEncoder().getVelocity());
    inputs.current = Amps.of(elevatorMotorLeft.getOutputCurrent());
  }

  private void setupMotors() {
    elevatorMotorLeft.configure(
        new SparkMaxConfig()
            .smartCurrentLimit(ElevatorConstants.kLimit)
            .inverted(ElevatorConstants.kInverted)
            .apply(
                new EncoderConfig()
                    .velocityConversionFactor(ElevatorConstants.kVelocityConversionFactor)
                    .positionConversionFactor(ElevatorConstants.kPositionConversionFactor)),
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
    elevatorMotorRight.configure(
        new SparkMaxConfig()
            .smartCurrentLimit(ElevatorConstants.kLimit)
            .inverted(ElevatorConstants.kInverted)
            .apply(
                new EncoderConfig()
                    .velocityConversionFactor(ElevatorConstants.kVelocityConversionFactor)
                    .positionConversionFactor(ElevatorConstants.kPositionConversionFactor))
            .follow(elevatorMotorLeft),
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
  }

  public void setVoltage(Voltage Volts) {
    elevatorMotorLeft.setVoltage(Volts);
  }
}
