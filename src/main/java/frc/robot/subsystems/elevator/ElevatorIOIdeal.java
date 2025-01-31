/* (C) Robolancers 2025 */
package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volt;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;

@Logged
// Ideal case for use when Elevator is disabled, doesn't actually control anything
public class ElevatorIOIdeal implements ElevatorIO {

  // Crate motor objects
  public TalonFX elevatorMotorLeft = new TalonFX(ElevatorConstants.kLeftMotorID);
  public TalonFX elevatorMotorRight = new TalonFX(ElevatorConstants.kRightMotorID);

  // updateInputs method updates the Inputs wkith the actual values from motors;
  // Called repeatedly in periodic method in Elevator
  public void updateInputs(ElevatorInputs inputs) {
    inputs.height = Meters.of(elevatorMotorLeft.getPosition().getValueAsDouble());
    inputs.velocity = MetersPerSecond.of(elevatorMotorLeft.getVelocity().getValueAsDouble());
    inputs.current = Amps.of(elevatorMotorLeft.getStatorCurrent().getValueAsDouble());
  }

  // Sets power to motors in Volts (Both motors bc Right motor has not been setup to follow left
  // motor)
  public void setVoltage(Voltage Volts) {
    elevatorMotorLeft.setVoltage(Volts.in(Volt));
    elevatorMotorRight.setVoltage(Volts.in(Volt));
  }

  // Sets encoder pos of left motor
  public void setEncoderPosition(Distance height) {
    elevatorMotorLeft.setPosition(height.in(Meters));
  }
}
