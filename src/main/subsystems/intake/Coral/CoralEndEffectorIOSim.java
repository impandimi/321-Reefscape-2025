/* (C) Robolancers 2025 */
package frc.robot.subsystems.coralendeffector;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// implementation of the CoralEndEffectorIO that controls the simulated coral end effector
@Logged
class CoralEndEffectorIOSim implements CoralEndEffectorIO {

  // Haven't talked with mechanical team yet so the values are 0
  public DCMotorSim motorSim =
      new DCMotorSim(LinearSystemId.createDCMotorSystem(DCMotor.getNEO(0), 0, 0), null, null);

public CoralEndEffectorIOSim(){
  SmartDashboard.putBoolean("HasCoral", false);
}

  @Override
  public void updateInputs(CoralEndEffectorInputs inputs) {
    inputs.voltage = Volts.of(motorSim.getInputVoltage());
    inputs.isBeamBreakBroken = SmartDashboard.getBoolean("HasCoral", false);
  }

  @Override
  public void setVoltage(Voltage voltage) {
    motorSim.setInputVoltage(voltage.in(Volts));
  }
}
