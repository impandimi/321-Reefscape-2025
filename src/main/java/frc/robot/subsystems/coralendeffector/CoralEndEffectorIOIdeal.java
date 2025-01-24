/* (C) Robolancers 2025 */
package frc.robot.subsystems.coralendeffector;

import static edu.wpi.first.units.Units.Volts;

public class CoralEndEffectorIOIdeal implements CoralEndEffectorIO {

  public CoralEndEffectorIOIdeal() {}

  // All input default values are off or false
  @Override
  public void updateInputs(CoralEndEffectorInputs inputs) {
    inputs.voltage = Volts.of(0);
    inputs.hasCoral = false;
    inputs.isBeamBreakBroken = false;
  }
}
