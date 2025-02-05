/* (C) Robolancers 2025 */
package frc.robot.subsystems.elevatorarm;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;

// Defines the inputs from the sensors that the ElevatorArm subsystem will receive
@Logged
public class ElevatorArmInputs {
  public Angle angle = Degrees.of(0); // the angle of the arm
  public AngularVelocity velocity; // the velocity of the arm
  public Current current; // the current draw of the arm
}
