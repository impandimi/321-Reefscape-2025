/* (C) Robolancers 2025 */
package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;

@Logged
// These are all the things that get "input" into the elevator class from encoders, motors, etc.
public class ElevatorInputs {
  public Distance height = Inches.of(0);
  public LinearVelocity velocity;
  public Current current;
}
