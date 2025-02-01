/* (C) Robolancers 2025 */
package frc.robot.subsystems.algaeIntakeClimb;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.units.measure.Voltage;

public class AlgaeIntakeClimbIOIdeal implements AlgaeIntakeClimbIO {
  // this class is for when the robot disables
  public static final AlgaeIntakeClimbConfig config = new AlgaeIntakeClimbConfig(0, 0, 0, 0);

  public void updateInputs(AlgaeIntakeClimbInputs inputs) {
    inputs.currentPivotAngle = Degrees.of(0);
    inputs.hasAlgae = false;
  }

  public void setPivotVoltage(Voltage volts) {}
}
