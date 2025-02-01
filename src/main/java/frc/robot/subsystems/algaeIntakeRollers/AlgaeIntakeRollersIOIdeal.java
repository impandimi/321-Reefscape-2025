/* (C) Robolancers 2025 */
package frc.robot.subsystems.algaeIntakeRollers;

import static edu.wpi.first.units.Units.RPM;

// class for when the robot disables
public class AlgaeIntakeRollersIOIdeal implements AlgaeIntakeRollersIO {
  public void updateInputs(AlgaeIntakeRollersInputs inputs) {
    inputs.rollerVelocity = RPM.of(0);
  }

  public void setRollerVoltage() {}
}
