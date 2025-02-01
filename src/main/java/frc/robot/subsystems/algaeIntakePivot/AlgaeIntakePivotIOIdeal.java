/* (C) Robolancers 2025 */
package frc.robot.subsystems.algaeIntakePivot;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.units.measure.Voltage;

public class AlgaeIntakePivotIOIdeal implements AlgaeIntakePivotIO {
  // this class is for when the robot disables
  public static final AlgaeIntakePivotConfig config = new AlgaeIntakePivotConfig(0, 0, 0, 0);

  public void updateInputs(AlgaeIntakePivotInputs inputs) {
    inputs.currentPivotAngle = Degrees.of(0);
    inputs.hasAlgae = false;
  }

  public void setPivotVoltage(Voltage volts) {}
}
