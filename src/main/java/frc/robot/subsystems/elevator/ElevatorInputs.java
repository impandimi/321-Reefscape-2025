package frc.robot.subsystems.elevator;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;

@Logged
public class ElevatorInputs {
    public Current current;
    public Distance height;
    public LinearVelocity velocity;
}