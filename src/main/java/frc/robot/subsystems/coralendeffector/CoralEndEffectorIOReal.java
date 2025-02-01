/* (C) Robolancers 2025 */
package frc.robot.subsystems.coralendeffector;

import static edu.wpi.first.units.Units.Millimeter;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Volts;

import com.playingwithfusion.TimeOfFlight;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.measure.Voltage;

// implementation of the CoralEndEffectorIO that controls the real coral end effector
@Logged
public class CoralEndEffectorIOReal implements CoralEndEffectorIO {

  public static CoralEndEffectorConfig config = new CoralEndEffectorConfig(0, 0, 0, 0);

  private SparkMax motor;
  private TimeOfFlight distSensor;

  public CoralEndEffectorIOReal() {
    this.motor = new SparkMax(CoralEndEffectorConstants.kMotorPort, MotorType.kBrushless);
    motor.configure(
        new SparkMaxConfig()
            .inverted(CoralEndEffectorConstants.kInvertedMotor)
            .smartCurrentLimit(CoralEndEffectorConstants.kCurrentLimit),
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    distSensor = new TimeOfFlight(CoralEndEffectorConstants.kTimeOfFlightId);
  }

  // sets voltage
  @Override
  public void setVoltage(Voltage voltage) {
    motor.setVoltage(voltage);
  }

  // updates inputs
  @Override
  public void updateInputs(CoralEndEffectorInputs inputs) {
    inputs.voltage = Volts.of(motor.getBusVoltage());
    inputs.hasCoral =
        distSensor.getRange() < CoralEndEffectorConstants.kDetectionRange.in(Millimeter);
    inputs.velocity = RPM.of(this.motor.getEncoder().getVelocity());
  }
}
