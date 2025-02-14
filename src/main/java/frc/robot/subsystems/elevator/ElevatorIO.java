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

  // update inputs from sensors
  default void updateInputs(ElevatorInputs inputs) {}

  // run the motor at a specific voltage
  default void setVoltage(Voltage volts) {}

  // set encoder position; doesn't run the motor
  default void setEncoderPosition(Distance position) {}

  // run elevator to position using onboard pid controller
  default void setPosition(Distance position) {}

  // set pid values of onboard pid controller
  default void setOnboardPID(ElevatorConfig config) {}

  default boolean atSetpoint() {
    return false;
  }
}
