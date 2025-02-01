/* (C) Robolancers 2025 */
package frc.robot.subsystems.algaeIntakeClimb;

import edu.wpi.first.units.measure.Voltage;

public interface AlgaeIntakeClimbIO {

  default void setPivotVoltage(Voltage volts) {}

  // sets voltage to the pivot
  default void updateInputs(AlgaeIntakeClimbInputs inputs) {}
  // to update inputs
}
