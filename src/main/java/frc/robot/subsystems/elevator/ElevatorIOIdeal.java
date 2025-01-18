/* (C) Robolancers 2025 */
package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.measure.Voltage;

@Logged
public class ElevatorIOIdeal implements ElevatorIO {

  public SparkMax elevatorMotorLeft =
      new SparkMax(ElevatorConstants.kLeftMotorID, MotorType.kBrushless);
  public SparkMax elevatorMotorRight =
      new SparkMax(ElevatorConstants.kRightMotorID, MotorType.kBrushless);

  public void updateInputs(ElevatorInputs inputs) {
    inputs.height = Meters.of(elevatorMotorLeft.getEncoder().getPosition());
    inputs.velocity = MetersPerSecond.of(elevatorMotorLeft.getEncoder().getVelocity());
    inputs.current = Amps.of(elevatorMotorLeft.getOutputCurrent());
  }

  public void setVoltage(Voltage Volts) {
    elevatorMotorLeft.setVoltage(Volts);
  }
}
