/* (C) Robolancers 2025 */
package frc.robot.subsystems.coralendeffector;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.Voltage;

public class CoralEndEffectorConstants {
  public static final int kMotorPort = 0;
  public static final boolean kInvertedMotor = false;
  public static final int kCurrentLimit = 40;
  public static final Voltage kIntakeVoltage = Volts.of(9);
  public static final Voltage kOuttakeVoltage = Volts.of(-9);
  public static final Voltage kStallVoltage = Volts.of(1.2);

  public static final int kBeamBreakPort = 0;

  public static final int kTouchSensorPort = 0;
}
