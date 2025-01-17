/* (C) Robolancers 2025 */
package frc.robot.subsystems.elevatorarm;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.measure.Voltage;

@Logged
public interface ElevatorArmIO {
  default void updateInputs(ElevatorArmInputs inputs) {}

  default void setVoltage(Voltage volts) {}
}
