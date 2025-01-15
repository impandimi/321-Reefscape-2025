package frc.robot.subsystems.elevator;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.measure.Voltage;

@Logged
public interface ElevatorIO {
    default void updateInputs(ElevatorInputs inputs) {}
    default void setPower(Voltage volts){}
    default void resetEncoder(){}
}
