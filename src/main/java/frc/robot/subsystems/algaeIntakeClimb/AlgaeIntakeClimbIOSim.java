/* (C) Robolancers 2025 */
package frc.robot.subsystems.algaeIntakeClimb;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class AlgaeIntakeClimbIOSim implements AlgaeIntakeClimbIO {

  public static final AlgaeIntakeClimbConfig config = new AlgaeIntakeClimbConfig(0, 0, 0, 0);

  private SingleJointedArmSim pivotSimLeft;

  public
  AlgaeIntakeClimbIOSim() { // configures a simulated arm with two pivot motors controlling one
    // pivot point
    pivotSimLeft =
        new SingleJointedArmSim(
            LinearSystemId.createSingleJointedArmSystem(
                DCMotor.getNEO(2),
                AlgaeIntakeClimbConstants.kPivotMOI,
                AlgaeIntakeClimbConstants.kPivotGearing),
            DCMotor.getNEO(2),
            AlgaeIntakeClimbConstants.kPivotGearing,
            AlgaeIntakeClimbConstants.kPivotLengthMeters.in(Meters),
            AlgaeIntakeClimbConstants.kPivotMinAngle.in(Radians),
            AlgaeIntakeClimbConstants.kPivotMaxAngle.in(Radians),
            true,
            AlgaeIntakeClimbConstants.kPivotStartingAngle.in(Radians));
    SmartDashboard.putBoolean("AlgaeIntakeClimbSim/HasAlgae", false);
  }

  public void setPivotVoltage(Voltage volts) {
    pivotSimLeft.setInputVoltage(volts.in(Volts));
  }

  public void updateInputs(AlgaeIntakeClimbInputs inputs) { // gets info to update inputs
    inputs.currentPivotAngle = Degrees.of(pivotSimLeft.getAngleRads());
    inputs.hasAlgae = SmartDashboard.getBoolean("AlgaeIntakeClimbSim/HasAlgae", false);
  }
}
