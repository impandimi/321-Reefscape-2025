/* (C) Robolancers 2025 */
package frc.robot.subsystems.coralendeffector;

import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.measure.Voltage;

import com.playingwithfusion.TimeOfFlight;

// implementation of the CoralEndEffectorIO that controls the real coral end effector
@Logged
public class CoralEndEffectorIOReal implements CoralEndEffectorIO {

  private SparkMax motor;
  private TimeOfFlight touchSensor;
  private TimeOfFlight beamBreak; // beambreak and touch sensor not in use right now

  public CoralEndEffectorIOReal() {
    this.motor = new SparkMax(CoralEndEffectorConstants.kMotorPort, MotorType.kBrushless);
    motor.configure(
        new SparkMaxConfig()
            .inverted(CoralEndEffectorConstants.kInvertedMotor)
            .smartCurrentLimit(CoralEndEffectorConstants.kCurrentLimit),
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);


    touchSensor = new TimeOfFlight(CoralEndEffectorConstants.kTouchSensorPort);
    beamBreak = new TimeOfFlight(CoralEndEffectorConstants.kTimeOfFlightID);
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
    inputs.hasCoral = touchSensor.getRange() < CoralEndEffectorConstants.kDetectionRange; // ask if this works ?
    inputs.isBeamBreakBroken = beamBreak.getRange() < CoralEndEffectorConstants.kDetectionRange;
  }
}
