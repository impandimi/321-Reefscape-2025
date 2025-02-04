/* (C) Robolancers 2025 */
package frc.robot.subsystems.algaeIntakePivot;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;

@Logged
public class AlgaeIntakePivotInputs {
  public Angle pivotAngle; // current pivot angle
  public AngularVelocity pivotVelocity; // current pivot velocity
}
