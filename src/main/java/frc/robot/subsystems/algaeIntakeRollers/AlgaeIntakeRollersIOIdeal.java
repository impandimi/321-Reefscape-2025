/* (C) Robolancers 2025 */
package frc.robot.subsystems.algaeIntakeRollers;

import static edu.wpi.first.units.Units.RPM;

// class for when the robot disables
import edu.wpi.first.epilogue.Logged;
@Logged
public class AlgaeIntakeRollersIOIdeal implements AlgaeIntakeRollersIO {
public static final AlgaeIntakeRollersConfig config = new AlgaeIntakeRollersConfig(0, 0, 0, 0);
  public void updateInputs(AlgaeIntakeRollersInputs inputs) {
    inputs.rollerVelocity = RPM.of(0);
    inputs.hasAlgae = false;
  }

  public void setRollerVoltage() {}
}
