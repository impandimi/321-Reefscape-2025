/* (C) Robolancers 2025 */
package frc.robot.subsystems.algaeIntakePivot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;

@Logged
public class AlgaeIntakePivotConstants {
  // TODO: get all these constants

  // motor, encoder, sensor IDs
  public static final int kPivotMotorLeftId = 0;
  public static final int kPivotMotorRightId = 0;
  public static final int kEncoderId = 0;

  // setpoint constants
  public static final Angle kPivotClimbThreshold =
      Degrees.of(135); // the angle to climb to; NOT a direct setpoint
  public static final Voltage kPivotClimbVoltage = Volts.of(8); // test

  public static final Angle kMinBlockedAngle = Degrees.of(20);
  public static final Angle kMaxBlockedAngle = Degrees.of(40);

  // physical constants
  public static final double kPivotGearing = 150; // ask mech
  public static final Distance kPivotLength = Meters.of(0.6); // ask CAD
  public static final Angle kPivotMinAngle = Degrees.of(-16); // find
  public static final Angle kPivotMaxAngle = Degrees.of(180); // find
  public static final Angle kPivotStartingAngle = Degrees.of(-16); // find
  public static final double kPivotMOI = 1; // ask stanley
  public static final Angle kPivotZeroOffsetAngle = Degrees.of(0); // find

  // motor configurations
  public static final boolean kLeftInverted = false;
  public static final boolean kRightInverted = false;
  public static final int kSmartCurrentLimit = 40;
  public static final double kPivotPositionConversionFactor = 360 / kPivotGearing;
  public static final double kPivotVelocityConversionFactor = kPivotPositionConversionFactor / 60;
  public static final Voltage kNominalVoltage = Volts.of(12);
}
