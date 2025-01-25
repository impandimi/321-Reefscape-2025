package frc.robot.subsystems.algaelow;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Algaelow extends SubsystemBase{

private AlgaelowIO io;
private AlgaelowInputs inputs;

private PIDController algaelowController;
private ArmFeedforward feedForward;

public Algaelow(AlgaelowIO io, AlgaelowConfig config){
    this.io = io;
    this.inputs = new AlgaelowInputs();

    PIDController algaelowController = new PIDController(config.kP(), config.kI(), config.kD());
    ArmFeedforward feedForward = new ArmFeedforward(0,config.kG(),0);
}

public static Algaelow create() {
    return RobotBase.isReal() ?
    new Algaelow(new AlgaelowIOReal(), AlgaelowIOReal.config) :
    new Algaelow(new AlgaelowIOSim(), AlgaelowIOSim.config);
}

public static Algaelow disable(){
    return new Algaelow(new AlgaelowIOIdeal(), AlgaelowIOIdeal.config);
}




}



