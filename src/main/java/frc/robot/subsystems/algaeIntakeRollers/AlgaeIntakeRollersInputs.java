/* (C) Robolancers 2025 */
package frc.robot.subsystems.algaeIntakerollers;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.measure.AngularVelocity;

@Logged
public class AlgaeIntakeRollersInputs {
  public AngularVelocity rollerVelocity; // speed that the rollers are spinning
  public boolean hasAlgae; // if the mechanism has algae
}
