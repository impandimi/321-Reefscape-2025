/* (C) Robolancers 2025 */
package frc.robot.subsystems.coralendeffector;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;

// list of constants for the coral end effector
public class CoralEndEffectorConstants {
  public static final int kMotorPort = 0;
  public static final boolean kInvertedMotor = false;
  public static final int kCurrentLimit = 40;
  public static final Voltage kIntakeVoltage = Volts.of(9);
  public static final Voltage kOuttakeVoltage = Volts.of(-9);
  public static final Voltage kStallVoltage = Volts.of(1.2);
  public static final double momentOfInertia = 1;
  public static final double gearing = 1;

  public static final Distance kDetectionRange = Meters.of(0);

  public static final int kTimeOfFlightId = 0;
}
