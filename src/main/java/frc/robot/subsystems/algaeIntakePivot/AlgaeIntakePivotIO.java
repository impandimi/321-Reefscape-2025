/* (C) Robolancers 2025 */
package frc.robot.subsystems.algaeIntakePivot;

import edu.wpi.first.units.measure.Voltage;

import edu.wpi.first.epilogue.Logged;
@Logged
public interface AlgaeIntakePivotIO {

  default void setPivotVoltage(Voltage volts) {}

  // sets voltage to the pivot
  default void updateInputs(AlgaeIntakePivotInputs inputs) {}
  // to update inputs
}
