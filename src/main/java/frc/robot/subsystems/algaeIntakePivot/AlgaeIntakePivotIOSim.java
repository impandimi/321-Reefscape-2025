/* (C) Robolancers 2025 */
package frc.robot.subsystems.algaeIntakePivot;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class AlgaeIntakePivotIOSim implements AlgaeIntakePivotIO {

  public static final AlgaeIntakePivotConfig config = new AlgaeIntakePivotConfig(0, 0, 0, 0);

  private SingleJointedArmSim pivotSimLeft;

  public
  AlgaeIntakePivotIOSim() { // configures a simulated arm with two pivot motors controlling one
    // pivot point
    pivotSimLeft =
        new SingleJointedArmSim(
            LinearSystemId.createSingleJointedArmSystem(
                DCMotor.getNEO(2),
                AlgaeIntakePivotConstants.kPivotMOI,
                AlgaeIntakePivotConstants.kPivotGearing),
            DCMotor.getNEO(2),
            AlgaeIntakePivotConstants.kPivotGearing,
            AlgaeIntakePivotConstants.kPivotLengthMeters.in(Meters),
            AlgaeIntakePivotConstants.kPivotMinAngle.in(Radians),
            AlgaeIntakePivotConstants.kPivotMaxAngle.in(Radians),
            true,
            AlgaeIntakePivotConstants.kPivotStartingAngle.in(Radians));
    SmartDashboard.putBoolean("AlgaeIntakeClimbSim/HasAlgae", false);
  }

  public void setPivotVoltage(Voltage volts) {
    pivotSimLeft.setInputVoltage(volts.in(Volts));
  }

  public void updateInputs(AlgaeIntakePivotInputs inputs) { // gets info to update inputs
    inputs.currentPivotAngle = Radians.of(pivotSimLeft.getAngleRads());
    inputs.hasAlgae = SmartDashboard.getBoolean("AlgaeIntakeClimbSim/HasAlgae", false);
  }
}
