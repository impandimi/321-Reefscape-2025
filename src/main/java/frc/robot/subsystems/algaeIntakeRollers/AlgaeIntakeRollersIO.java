/* (C) Robolancers 2025 */
package frc.robot.subsystems.algaeIntakeRollers;

import edu.wpi.first.units.measure.Voltage;

public interface AlgaeIntakeRollersIO {

  default void updateInputs(AlgaeIntakeRollersInputs inputs) {} // updates inputs

  default void setRollerVoltage(Voltage volts) {} // sets roller voltage
}
