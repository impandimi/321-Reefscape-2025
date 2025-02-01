/* (C) Robolancers 2025 */
package frc.robot.subsystems.algaeIntakeRollers;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class AlgaeIntakeRollersIOSim implements AlgaeIntakeRollersIO {

  private DCMotorSim rollerSim;

  public AlgaeIntakeRollersIOSim() {
    rollerSim = // creates and configures a single motor sim to represent the mechanism's rollers
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                DCMotor.getNEO(1),
                AlgaeIntakeRollersConstants.kRollerMOI,
                AlgaeIntakeRollersConstants.kRollerGearing),
            DCMotor.getNEO(1));
    SmartDashboard.putBoolean("AlgaeIntakeClimbSim/HasAlgae", false);
  }

  public void setRollerVoltage(Voltage volts) {
    rollerSim.setInputVoltage(volts.in(Volts));
  }

  public void updateInputs(AlgaeIntakeRollersInputs inputs) {
    inputs.rollerVelocity = RPM.of(rollerSim.getAngularVelocityRadPerSec()); // gets input info
    inputs.hasAlgae = SmartDashboard.getBoolean("AlgaeIntakeClimbSim/HasAlgae", false);
  }
}
