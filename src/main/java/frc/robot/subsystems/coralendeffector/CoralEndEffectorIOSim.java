/* (C) Robolancers 2025 */
package frc.robot.subsystems.coralendeffector;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

// implementation of the CoralEndEffectorIO that controls the simulated coral end effector
@Logged
class CoralEndEffectorIOSim implements CoralEndEffectorIO {
  // Constant values for the simulation of the coral end effector

  // public static final CoralEndEffectorConfig config = new CoralEndEffectorConfig(4, 0, 0, 0.9,
  // 0); <- do we need this / do we need config at all?

  public DCMotorSim simulation =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(
              DCMotor.getNEO(2),
              CoralEndEffectorConstants.momentOfInertia,
              CoralEndEffectorConstants.gearing),
          DCMotor.getNEO(2));

  @Override
  public void updateInputs(CoralEndEffectorInputs inputs) {
    simulation.update(0.02);
    inputs.voltage = Volts.of(simulation.getInputVoltage());
    inputs.isBeamBreakBroken = false;
  }

  @Override
  public void setVoltage(Voltage voltage) {
    simulation.setInputVoltage(voltage.in(Volts));
  }
}
