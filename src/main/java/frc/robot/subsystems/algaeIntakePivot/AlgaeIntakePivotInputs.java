/* (C) Robolancers 2025 */
package frc.robot.subsystems.algaeIntakePivot;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.measure.Angle;

@Logged
public class AlgaeIntakePivotInputs {
  public Angle pivotAngle = Degrees.of(0); // current pivot angle
}
