package frc.robot.subsystems.algaeIntakeClimb;

import edu.wpi.first.units.measure.Voltage;

public interface AlgaeIntakeClimbIO {
    
default void setPivotPower(Voltage volts){}
default void setRollerPower(Voltage volts){}
default void updateInputs(AlgaeIntakeClimbInputs inputs){}

}
