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
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.measure.Voltage;

/** Implementation of the ElevatorArmIO that controls a real ElevatorArm */
@Logged
public class ElevatorArmIOReal implements ElevatorArmIO {

  // tuning config for the ElevatorArmIOReal
  public static final ElevatorArmConfig config = new ElevatorArmConfig(0, 0, 0, 0, 0, 0);

  // the motor that is controlling the arm
  private SparkMax armMotor =
      new SparkMax(ElevatorArmConstants.kElevatorArmId, MotorType.kBrushless);

  public ElevatorArmIOReal() {
    // setup arm motor

    armMotor.configure(
        new SparkMaxConfig() // config for basic motor stuff
            .smartCurrentLimit(ElevatorArmConstants.kCurrentLimit)
            .voltageCompensation(ElevatorArmConstants.kNominalVoltage)
            .idleMode(IdleMode.kBrake)
            .inverted(ElevatorArmConstants.kInverted)
            .apply(
                new EncoderConfig() // config for relative encoder
                    .positionConversionFactor(ElevatorArmConstants.kPositionConversionFactor)
                    .velocityConversionFactor(ElevatorArmConstants.kVelocityConversionFactor))
            .apply(
                new AbsoluteEncoderConfig() // config for absolute encoder
                    .positionConversionFactor(ElevatorArmConstants.kAbsPositionConversionFactor)
                    .velocityConversionFactor(ElevatorArmConstants.kAbsVelocityConversionFactor)),
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
  }

  // update inputs from the arm motor
  public void updateInputs(ElevatorArmInputs inputs) {
    inputs.angle = Degrees.of(armMotor.getEncoder().getPosition());
    inputs.velocity = DegreesPerSecond.of(armMotor.getEncoder().getVelocity());
    inputs.current = Amps.of(armMotor.getOutputCurrent());
  }

  // set voltage to the arm motor
  public void setVoltage(Voltage volts) {
    armMotor.setVoltage(volts);
  }
}
