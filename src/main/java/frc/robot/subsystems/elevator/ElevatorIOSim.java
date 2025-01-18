/* (C) Robolancers 2025 */
package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Amp;
import static edu.wpi.first.units.Units.Kilogram;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;

@Logged
public class ElevatorIOSim implements ElevatorIO {
  public static final ElevatorConfig config = new ElevatorConfig(4, 0, 0, 0.9, 0);

  public ElevatorSim simMotor =
      new ElevatorSim(
          DCMotor.getNEO(2),
          ElevatorConstants.kElevatorGearing,
          ElevatorConstants.kElevatorCarriageMass.in(Kilogram),
          ElevatorConstants.kElevatorDrumRadius.in(Meters),
          ElevatorConstants.kElevatorMinimumHeight.in(Meters),
          ElevatorConstants.kElevatorMaximumHeight.in(Meters),
          true,
          ElevatorConstants.kElevatorStartingHeight.in(Meters));

  public void updateInputs(ElevatorInputs inputs) {
    simMotor.update(0.2);
    inputs.height = Meters.of(simMotor.getPositionMeters());
    inputs.velocity = MetersPerSecond.of(simMotor.getVelocityMetersPerSecond());
    inputs.current = Amp.of(simMotor.getCurrentDrawAmps());
  }

  public void setVoltage(Voltage volts) {
    simMotor.setInputVoltage(volts.in(Volts));
  }
}
