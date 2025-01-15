package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;

public class ElevatorIOSim implements ElevatorIO{
    public static final ElevatorConfig config = new ElevatorConfig(0,0,0,0);
    
    private final ElevatorSim elevatorSim;
    public ElevatorIOSim() {
        elevatorSim = new ElevatorSim(
          DCMotor.getNEO(1).withReduction(ElevatorConstants.kElevatorGearing),
          ElevatorConstants.kElevatorGearing,
          ElevatorConstants.kCarriageMass,
          ElevatorConstants.kElevatorDrumRadius,
          ElevatorConstants.kMinElevatorHeightMeters,
          ElevatorConstants.kMaxElevatorHeightMeters,
          true,
          0,
          0.01,
          0.0);
    }

    public void setPower(Voltage volts){   
        elevatorSim.setInputVoltage(volts.in(Volts));
    }

    public void updateInputs(ElevatorInputs inputs){
        inputs.height = Meters.of(elevatorSim.getPositionMeters());
        inputs.velocity = MetersPerSecond.of(elevatorSim.getPositionMeters());
        inputs.current = Amps.of(elevatorSim.getCurrentDrawAmps());
    }

    public void resetEncoder(){
        elevatorSim.setState(0, 0);
    }
}
