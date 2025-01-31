/* (C) Robolancers 2025 */
package frc.robot.subsystems.algaeIntakeClimb;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.units.measure.Voltage;

public class AlgaeIntakeClimbIOIdeal implements AlgaeIntakeClimbIO {

  public static final AlgaeIntakeClimbConfig config = new AlgaeIntakeClimbConfig(0, 0, 0, 0);

  public void updateInputs(AlgaeIntakeClimbInputs inputs) {
    inputs.pivotVelocity = RPM.of(0);
    inputs.currentPivotAngle = Degrees.of(0);
    inputs.hasAlgae = false;
    inputs.rollerVelocity = RPM.of(0);
  }

  public void setPivotVoltage(Voltage volts) {}

  public void setRollerVoltage(Voltage volts) {}
}
