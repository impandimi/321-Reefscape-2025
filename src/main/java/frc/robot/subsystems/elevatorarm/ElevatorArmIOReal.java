/* (C) Robolancers 2025 */
package frc.robot.subsystems.elevatorarm;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.AbsoluteEncoderConfig;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.measure.Voltage;

@Logged
public class ElevatorArmIOReal implements ElevatorArmIO {

  public static final ElevatorArmConfig config = new ElevatorArmConfig(0, 0, 0, 0, 0, 0, 0);

  private SparkMax armMotor =
      new SparkMax(ElevatorArmConstants.kElevatorArmId, MotorType.kBrushless);

  public ElevatorArmIOReal() {
    // setup arm motor

    EncoderConfig encConfig =
        new EncoderConfig()
            .positionConversionFactor(ElevatorArmConstants.kPositionConversionFactor)
            .velocityConversionFactor(ElevatorArmConstants.kVelocityConversionFactor);

    AbsoluteEncoderConfig absEncConfig =
        new AbsoluteEncoderConfig()
            .positionConversionFactor(ElevatorArmConstants.kAbsPositionConversionFactor)
            .velocityConversionFactor(ElevatorArmConstants.kAbsVelocityConversionFactor);

    SparkBaseConfig config =
        new SparkMaxConfig()
            .smartCurrentLimit(ElevatorArmConstants.kCurrentLimit)
            .voltageCompensation(ElevatorArmConstants.kNominalVoltage)
            .idleMode(IdleMode.kBrake)
            .inverted(ElevatorArmConstants.kInverted)
            .apply(encConfig)
            .apply(absEncConfig);

    armMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void updateInputs(ElevatorArmInputs inputs) {
    inputs.angle = Degrees.of(armMotor.getEncoder().getPosition());
    inputs.velocity = DegreesPerSecond.of(armMotor.getEncoder().getVelocity());
    inputs.current = Amps.of(armMotor.getOutputCurrent());
  }

  public void setVoltage(Voltage volts) {
    armMotor.setVoltage(volts);
  }
}
