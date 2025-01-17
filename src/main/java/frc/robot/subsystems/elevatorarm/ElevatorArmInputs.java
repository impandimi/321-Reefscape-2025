/* (C) Robolancers 2025 */
package frc.robot.subsystems.elevatorarm;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;

@Logged
public class ElevatorArmInputs {
  public Angle angle;
  public AngularVelocity velocity;
  public Current current;
}
