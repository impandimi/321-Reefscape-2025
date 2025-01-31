/* (C) Robolancers 2025 */
package frc.robot.subsystems.algaeIntakeClimb;

import edu.wpi.first.units.measure.Voltage;

public interface AlgaeIntakeClimbIO {

  default void setPivotVoltage(Voltage volts) {}

  default void setRollerVoltage(Voltage volts) {}

  default void updateInputs(AlgaeIntakeClimbInputs inputs) {}
}
