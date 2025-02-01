/* (C) Robolancers 2025 */
package frc.robot.subsystems.algaeIntakeRollers;

import edu.wpi.first.units.measure.AngularVelocity;

import edu.wpi.first.epilogue.Logged;
@Logged
public class AlgaeIntakeRollersInputs {
  public AngularVelocity rollerVelocity; // speed that the rollers are spinning
  public boolean hasAlgae; // if the mechanism has algae
}
