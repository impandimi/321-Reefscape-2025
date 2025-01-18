/* (C) Robolancers 2025 */
package frc.robot.subsystems.coralendeffector;

public class CoralEndEffectorIOIdeal implements CoralEndEffectorIO {

  public CoralEndEffectorIOIdeal() {}

  // coralEndEffectorInputs
  @Override
  public void updateInputs(CoralEndEffectorInputs inputs) {
    inputs.voltage = 0;
    inputs.power = 0;
    inputs.hasCoral = false;
    inputs.isBroken = false;
  }
}
