/* (C) Robolancers 2025 */
package frc.robot.subsystems.algaeIntakeClimb;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class AlgaeIntakeClimbIOSim implements AlgaeIntakeClimbIO {

  public static final AlgaeIntakeClimbConfig config = new AlgaeIntakeClimbConfig(0, 0, 0, 0);

  DCMotorSim rollerSim;
  SingleJointedArmSim pivotSimLeft;
  SingleJointedArmSim pivotSimRight;

  public AlgaeIntakeClimbIOSim() {
    rollerSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                DCMotor.getNEO(1),
                AlgaeIntakeClimbConstants.kRollerMOI,
                AlgaeIntakeClimbConstants.kGearing),
            DCMotor.getNEO(1));
    pivotSimLeft =
        new SingleJointedArmSim(
            LinearSystemId.createSingleJointedArmSystem(
                DCMotor.getNEO(1),
                AlgaeIntakeClimbConstants.kSimJKgSquaredMeters,
                AlgaeIntakeClimbConstants.kGearing),
            DCMotor.getNEO(1),
            AlgaeIntakeClimbConstants.kGearing,
            AlgaeIntakeClimbConstants.kArmLengthMeters.in(Meters),
            AlgaeIntakeClimbConstants.kSimMinAngle.in(Radians),
            AlgaeIntakeClimbConstants.kSimMaxAngle.in(Radians),
            true,
            AlgaeIntakeClimbConstants.kSimStartingAngle.in(Radians));
    SmartDashboard.putBoolean("AlgaeIntakeClimbSim/HasAlgae", false);
    pivotSimRight =
        new SingleJointedArmSim(
            LinearSystemId.createSingleJointedArmSystem(
                DCMotor.getNEO(1),
                AlgaeIntakeClimbConstants.kSimJKgSquaredMeters,
                AlgaeIntakeClimbConstants.kGearing),
            DCMotor.getNEO(1),
            AlgaeIntakeClimbConstants.kGearing,
            AlgaeIntakeClimbConstants.kArmLengthMeters.in(Meters),
            AlgaeIntakeClimbConstants.kSimMinAngle.in(Radians),
            AlgaeIntakeClimbConstants.kSimMaxAngle.in(Radians),
            true,
            AlgaeIntakeClimbConstants.kSimStartingAngle.in(Radians));
    SmartDashboard.putBoolean("AlgaeIntakeClimbSim/HasAlgae", false);
  }

  public void setPivotVoltage(Voltage volts) {
    pivotSimLeft.setInputVoltage(volts.in(Volts));
    pivotSimRight.setInputVoltage(volts.in(Volts));
  }

  public void setRollerVoltage(Voltage volts) {
    rollerSim.setInputVoltage(volts.in(Volts));
  }

  public void updateInputs(AlgaeIntakeClimbInputs inputs) {
    inputs.currentPivotAngle = Degrees.of(pivotSimLeft.getAngleRads());
    inputs.pivotVelocity = RPM.of(pivotSimLeft.getVelocityRadPerSec());
    inputs.rollerVelocity = RPM.of(rollerSim.getAngularVelocityRadPerSec());
    inputs.hasAlgae = SmartDashboard.getBoolean("AlgaeIntakeClimbSim/HasAlgae", false);
  }
}
