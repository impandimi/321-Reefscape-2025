/* (C) Robolancers 2025 */
package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;

@Logged
// Ideal case where Elevator exists in a perfect world (will get better explanation here soon lol)
public class ElevatorIOIdeal implements ElevatorIO {

  // Crate motor objects
  public SparkMax elevatorMotorLeft =
      new SparkMax(ElevatorConstants.kLeftMotorID, MotorType.kBrushless);
  public SparkMax elevatorMotorRight =
      new SparkMax(ElevatorConstants.kRightMotorID, MotorType.kBrushless);

  // updateInputs method updates the Inputs wkith the actual values from motors;
  // Called repeatedly in periodic method in Elevator
  public void updateInputs(ElevatorInputs inputs) {
    inputs.height = Meters.of(elevatorMotorLeft.getEncoder().getPosition());
    inputs.velocity = MetersPerSecond.of(elevatorMotorLeft.getEncoder().getVelocity());
    inputs.current = Amps.of(elevatorMotorLeft.getOutputCurrent());
  }

  // Sets power to motors in Volts (Both motors bc Right motor has not been setup to follow left
  // motor)
  public void setVoltage(Voltage Volts) {
    elevatorMotorLeft.setVoltage(Volts);
    elevatorMotorRight.setVoltage(Volts);
  }

  // Sets encoder pos of left motor
  public void setEncoderPosition(Distance height) {
    elevatorMotorLeft.getEncoder().setPosition(height.in(Meters));
  }
}
