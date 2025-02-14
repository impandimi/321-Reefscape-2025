/* (C) Robolancers 2025 */
package frc.robot.subsystems.elevator;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;

@Logged
// Template for all IO
// all IO must have ALL of these methods
/*Names are pretty self explanantory, but regardless:
updateInputs
  updates the Elevator Values that onew would get from, say, the encoder
setVoltage
  sets power of elevator via voltage
setEncoderPosition
  ...Sets the encoder position
*/
public interface ElevatorIO {

  default void updateInputs(ElevatorInputs inputs) {}

  default void setVoltage(Voltage volts) {}

  default void setEncoderPosition(Distance position) {}

  default void setPosition(Distance position) {}

  default void setOnboardPID(ElevatorConfig config) {}
}
