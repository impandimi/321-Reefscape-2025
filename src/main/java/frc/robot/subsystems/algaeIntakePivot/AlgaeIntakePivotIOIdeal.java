/* (C) Robolancers 2025 */
package frc.robot.subsystems.algaeIntakepivot;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.measure.Voltage;

@Logged
public class AlgaeIntakePivotIOIdeal implements AlgaeIntakePivotIO {
  // this class is for when the robot disables
  public static final AlgaeIntakePivotConfig config = new AlgaeIntakePivotConfig(0, 0, 0, 0);

  public void updateInputs(AlgaeIntakePivotInputs inputs) {
    inputs.pivotAngle = Degrees.of(0);
  }

  public void setPivotVoltage(Voltage volts) {}
}
