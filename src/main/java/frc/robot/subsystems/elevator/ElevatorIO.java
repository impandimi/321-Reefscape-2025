/* (C) Robolancers 2025 */
package frc.robot.subsystems.elevator;

import edu.wpi.first.units.measure.Voltage;

public interface ElevatorIO {
  default void updateInputs(ElevatorInputs inputs) {}

  default void setVoltage(Voltage volts) {}
  // default void setEncoderPosition(Distance position) {}

}
