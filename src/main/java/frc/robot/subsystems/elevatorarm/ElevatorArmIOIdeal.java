/* (C) Robolancers 2025 */
package frc.robot.subsystems.elevatorarm;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.measure.Voltage;

/**
 * Implementation of the ElevatorArmIO that does NOT control anything This is used for easily
 * disabling the arm
 */
@Logged
public class ElevatorArmIOIdeal implements ElevatorArmIO {
  public static final ElevatorArmConfig config = new ElevatorArmConfig(0, 0, 0, 0, 0, 0);

  public void updateInputs(ElevatorArmInputs inputs) {
    inputs.angle = Radians.of(0);
    inputs.velocity = RadiansPerSecond.of(0);
    inputs.current = Amps.of(0);
  }

  public void setVoltage(Voltage volts) {}
}
