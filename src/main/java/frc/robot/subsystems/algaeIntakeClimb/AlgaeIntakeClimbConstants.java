/* (C) Robolancers 2025 */
package frc.robot.subsystems.algaeIntakeClimb;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;

public class AlgaeIntakeClimbConstants {

  public static final Angle kPivotIntakeAngle = Degrees.of(0); // test
  public static final Angle kPivotOuttakeAngle = Degrees.of(90); // test
  public static final Voltage kPivotClimbVoltage = Volts.of(2); // test
  public static final Angle kPivotClimbAngle = Degrees.of(0); // test
  // TODO: all values are placeholders
  public static final boolean kPivotInverted = false;
  public static final int kSmartCurrentLimit = 40;
  public static final double kPivotVelocityConversionFactor = 0; // ask mech
  public static final double kPivotPositionConversionFactor = 0; // ask mech
  public static final Voltage kPivotHangVoltage = Volts.of(0); // test
  public static final Angle kPivotFloorAngle = Degrees.of(180); // test
  public static final double kPivotGearing = 0; // ask mech
  public static final Distance kPivotLengthMeters = Meters.of(0); // ask CAD
  public static final Angle kPivotMinAngle = Degrees.of(0); // test
  public static final Angle kPivotMaxAngle = Degrees.of(0); // test
  public static final Angle kPivotStartingAngle = Degrees.of(0); // test
  public static final Voltage kNominalVoltage = Volts.of(12);
  public static final double kPivotMOI = 0; // ask stanley
  public static final Angle kPivotZeroOffsetAngle = Degrees.of(0); // test

  // TODO: also need the values for:
  // pid/feedforward config(tuning)
  // motor and digital input device id
}
