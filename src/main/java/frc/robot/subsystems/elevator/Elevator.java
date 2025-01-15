package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

@Logged
public class Elevator extends SubsystemBase {

public ElevatorIO io;
public ElevatorInputs inputs;

public PIDController elevatorController;
public ElevatorFeedforward elevatorFeedforward;

public Elevator(ElevatorIO io, ElevatorConfig config) {
    this.io = io;
    this.inputs = new ElevatorInputs();
    this.elevatorController = new PIDController(config.kP(), config.kI(), config.kD());
    this.elevatorFeedforward = new ElevatorFeedforward(0,config.kG(),0); 
}

public static Elevator create() {
        return RobotBase.isReal() ? 
            new Elevator(new ElevatorIOReal(), ElevatorIOReal.config):
            new Elevator(new ElevatorIOSim(), ElevatorIOSim.config); 
    }

    
public static Elevator disable() {
        return new Elevator(new ElevatorIOIdeal(), ElevatorIOIdeal.config); 
    }


public Command goToHeight(Distance desiredHeightMeters){
    return run(()->{
        double currentHeight = inputs.height.in(Meters);
        double desiredHeight = desiredHeightMeters.in(Meters);
        double PIDOutput = elevatorController.calculate(currentHeight, desiredHeight);
        Voltage PIDOutputVoltage = Volts.of(PIDOutput);
        io.setPower(PIDOutputVoltage);
    });

}


public Command goToStation(){
    return goToHeight(ElevatorConstants.kstationHeight);
}

public Command goToL1(){
    return goToHeight(ElevatorConstants.kL1Height);
}

public Command goToL2(){
    return goToHeight(ElevatorConstants.kL2Height);
}

public Command goToL3(){
    return goToHeight(ElevatorConstants.kL3Height);
}

public Command goToL4(){
    return goToHeight(ElevatorConstants.kL4Height);
}

public Command goToTravel(){
    return goToHeight(ElevatorConstants.kTravelHeight);
}

@Override
public void periodic(){
    io.updateInputs(inputs);
}


public Command voltageHoming(){
    return run(()->{
            io.setPower(ElevatorConstants.kVoltageHoming);
    }).until(() -> inputs.current.in(Amps) > 40).andThen(io::resetEncoder).finallyDo(()->{io.setPower(Volts.of(0));});
}
}
