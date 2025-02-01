/* (C) Robolancers 2025 */
package frc.robot.subsystems.elevatorarm;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

/** Implementation of the ElevatorArmIO that controls a simulated ElevatorArm */
@Logged
public class ElevatorArmIOSim implements ElevatorArmIO {

  // tuning config for the ElevatorArmIOSim
  public static final ElevatorArmConfig config = new ElevatorArmConfig(1, 0, 0.05, 4.07, 0);

  // simulated instance of the elevator arm
  private SingleJointedArmSim simMotor =
      new SingleJointedArmSim(
          DCMotor.getNEO(1),
          ElevatorArmConstants.kElevatorArmGearing,
          ElevatorArmConstants.kElevatorArmMOI,
          ElevatorArmConstants.kElevatorArmLength.in(Meters),
          ElevatorArmConstants.kMinAngle.in(Degrees),
          ElevatorArmConstants.kMaxAngle.in(Degrees),
          true,
          ElevatorArmConstants.kMinAngle.in(Degrees));

  // update inputs from the arm simulation
  public void updateInputs(ElevatorArmInputs inputs) {
    simMotor.update(0.02);
    inputs.angle = Radians.of(simMotor.getAngleRads());
    inputs.velocity = RadiansPerSecond.of(simMotor.getVelocityRadPerSec());
    inputs.current = Amps.of(simMotor.getCurrentDrawAmps());
  }

  // set voltage to the arm simulation
  public void setVoltage(Voltage volts) {
    simMotor.setInputVoltage(volts.in(Volts));
  }
}
