/* (C) Robolancers 2025 */
package frc.robot.subsystems.coralendeffector;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.measure.Voltage;

// below are inputs that the coral end effector will receive
@Logged
public class CoralEndEffectorInputs {
  public Voltage voltage;
  public boolean hasCoral;
  public boolean isBeamBreakBroken;
}
