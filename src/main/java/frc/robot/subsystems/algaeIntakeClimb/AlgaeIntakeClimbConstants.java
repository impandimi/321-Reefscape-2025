/* (C) Robolancers 2025 */
package frc.robot.subsystems.algaeIntakeClimb;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;

public class AlgaeIntakeClimbConstants {

  public static final Angle kIntakeAngle = Degrees.of(0);
  public static final Voltage kRollerIntakePower = Volts.of(3);
  public static final Voltage kRollerOuttakePower = Volts.of(-3);
  public static final Angle kPivotOuttakeAngle = Degrees.of(90);
  public static final Voltage kPivotClimbPower = Volts.of(2);
  public static final Angle kPivotClimbAngle = Degrees.of(0);
  public static final Voltage kPivotUnclimbPower = Volts.of(-0.1);
  // TODO: all values are placeholders
  public static final boolean kInverted = false;
  public static final int kSmartCurrentLimit = 40;
  public static final double kVelocityConversionFactor = 90;
  public static final double kPositionConversionFactor = 0;
  public static final Voltage kPivotHangPower = Volts.of(0);
  public static final Angle kPivotFloorAngle = Degrees.of(180);
  public static final double kSimJKgSquaredMeters = 0;
  public static final double kGearing = 0;
  public static final Distance kArmLengthMeters = Meters.of(0);
  public static final Angle kSimMinAngle = Degrees.of(0);
  public static final Angle kSimMaxAngle = Degrees.of(0);
  public static final Angle kSimStartingAngle = Degrees.of(0);
  public static final Angle kBeamBreakSimAngle = Degrees.of(0);
  public static final Voltage kNominalVoltage = Volts.of(12);
  public static final double kRollerMOI = 0;

  // TODO: also need the values for:
  // pid/feedforward config(tuning)
  // motor and sensor device id
  //
}
