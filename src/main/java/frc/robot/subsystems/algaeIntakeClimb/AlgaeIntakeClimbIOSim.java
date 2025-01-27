package frc.robot.subsystems.algaeIntakeClimb;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class AlgaeIntakeClimbIOSim implements AlgaeIntakeClimbIO{
    
    public static final AlgaeIntakeClimbConfig config = new AlgaeIntakeClimbConfig(4,3,2,1);

DCMotorSim rollerSim;
SingleJointedArmSim pivotSim;

public AlgaeIntakeClimbIOSim(){
    rollerSim = new DCMotorSim(LinearSystemId.createDCMotorSystem(0,0), DCMotor.getNEO(1));
    pivotSim = new SingleJointedArmSim(LinearSystemId.createSingleJointedArmSystem(DCMotor.getNEO(1), AlgaeIntakeClimbConstants.kSimJKgSquaredMeters, AlgaeIntakeClimbConstants.kSimGearing), DCMotor.getNEO(1), AlgaeIntakeClimbConstants.kSimGearing, AlgaeIntakeClimbConstants.kSimArmLengthMeters, AlgaeIntakeClimbConstants.kSimMinAngleRads, AlgaeIntakeClimbConstants.kSimMaxAngleRads, true, AlgaeIntakeClimbConstants.kSimStartingAngleRads);
    SmartDashboard.putBoolean("AlgaeIntakeClimbSim/HasAlgae", false); 
}

public void setPivotPower(Voltage volts){
pivotSim.setInputVoltage(volts.in(Volts));
}

public void setRollerPower(Voltage volts){
rollerSim.setInputVoltage(volts.in(Volts));
}

public void updateInputs(AlgaeIntakeClimbInputs inputs){
inputs.currentAngle = pivotSim.getAngleRads();
inputs.velocity = pivotSim.getVelocityRadPerSec();
inputs.rollerVelocity = rollerSim.getAngularVelocityRadPerSec();
inputs.rollerAngle = rollerSim.getAngularPositionRad();
inputs.hasAlgae = SmartDashboard.getBoolean("AlgaeIntakeClimbSim/HasAlgae", false);
}
}
