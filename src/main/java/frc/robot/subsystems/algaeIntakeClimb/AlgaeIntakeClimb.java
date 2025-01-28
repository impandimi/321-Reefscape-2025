package frc.robot.subsystems.algaeIntakeClimb;

import static edu.wpi.first.units.Units.Volts;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
// subsystem
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AlgaeIntakeClimb extends SubsystemBase{

private AlgaeIntakeClimbIO io;
private AlgaeIntakeClimbInputs inputs;

private PIDController algaeIntakeClimbController;
private ArmFeedforward feedForward;

public AlgaeIntakeClimb(AlgaeIntakeClimbIO io, AlgaeIntakeClimbConfig config){
    this.io = io;
    this.inputs = new AlgaeIntakeClimbInputs();

    algaeIntakeClimbController = new PIDController(config.kP(), config.kI(), config.kD());
    feedForward = new ArmFeedforward(0,config.kG(),0);
}

public static AlgaeIntakeClimb create() {
    return RobotBase.isReal() ?
    new AlgaeIntakeClimb(new AlgaeIntakeClimbIOReal(), AlgaeIntakeClimbIOReal.config) :
    new AlgaeIntakeClimb(new AlgaeIntakeClimbIOSim(), AlgaeIntakeClimbIOSim.config);
}

public static AlgaeIntakeClimb disable(){
    return new AlgaeIntakeClimb(new AlgaeIntakeClimbIOIdeal(), AlgaeIntakeClimbIOIdeal.config);
}

public void goToAngle(double desiredAngle){
    io.setPivotPower(
        Volts.of(
            feedForward.calculate(desiredAngle, 0, 0) 
          + algaeIntakeClimbController.calculate(inputs.currentAngle, desiredAngle)));
}

public void hang(Voltage volts){
    io.setPivotPower(volts);
}

public void spinRollers(Voltage volts){
    io.setRollerPower(volts);
}

//TODO:be careful if rollers need to start slightly after pivot

public Command outtake(){
    return run(()-> spinRollers(AlgaeIntakeClimbConstants.kouttakePower)) 
            .alongWith(run(()-> goToAngle(AlgaeIntakeClimbConstants.kouttakeAngle)))
            .until(() -> !inputs.hasAlgae);
}

public Command intake(){
    return run(()-> spinRollers(AlgaeIntakeClimbConstants.kintakePower))
            .alongWith(run(()-> goToAngle(AlgaeIntakeClimbConstants.kintakeAngle)))
            .until(() -> inputs.hasAlgae);
}

public Command floor(){ // to get into climbing position
    return run(()->goToAngle(AlgaeIntakeClimbConstants.kfloorAngle));
}

public Command climb(){
    return run(()->{
        if (inputs.currentAngle > AlgaeIntakeClimbConstants.kclimbMotorPosition){
            hang(AlgaeIntakeClimbConstants.kclimbPower);
        } else hang(Volts.of(0));
    });
}

public Command unclimb(){
    return run(()-> hang(AlgaeIntakeClimbConstants.kunclimbPower));
}

public Command moveAlgaeIntakeClimb(Supplier<Voltage> rollerVolts, DoubleSupplier pivotAngle){
    return run(()-> spinRollers(rollerVolts.get()))
            .alongWith(run(()-> goToAngle(pivotAngle.getAsDouble())))
            .until(() -> inputs.hasAlgae || inputs.currentAngle <= AlgaeIntakeClimbConstants.kclimbMotorPosition);
}

@Override
public void periodic() {
    io.updateInputs(inputs);
}

}








