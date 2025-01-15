package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.units.measure.Voltage;

public class ElevatorIOIdeal implements ElevatorIO {
    public static final ElevatorConfig config = new ElevatorConfig(0,0,0,0);
    

    public void updateInputs(ElevatorInputs inputs) {
        inputs.current = Amps.of(0);
        inputs.velocity = MetersPerSecond.of(0);
        inputs.height = Meters.of(0);
    }
    public void setPower(Voltage volts){}
    public void resetEncoder(){}
}
