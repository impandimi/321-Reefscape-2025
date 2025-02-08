/* (C) Robolancers 2025 */
package frc.robot.subsystems.algaeIntakePivot;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;

@Logged
public class AlgaeIntakePivotConstants {
  // TODO: get all these constants

  // motor, encoder, sensor IDs
  public static final int kPivotMotorLeftId = 0;
  public static final int kPivotMotorRightId = 0;

  // physical constants
  public static final double kPivotGearing = 150;
  public static final Distance kPivotLength = Meters.of(0.6);
  public static final Angle kPivotMinAngle = Degrees.of(0);
  public static final Angle kPivotMaxAngle = Degrees.of(200);
  public static final Angle kPivotStartingAngle = Degrees.of(0);
  public static final double kPivotMOI = 1; // TODO: find
  public static final Angle kPivotZeroOffsetAngle = Degrees.of(0);

  // setpoint constants
  public static final Angle kPivotClimbThreshold = Degrees.of(45); // to be tuned
  public static final Voltage kPivotClimbVoltage = Volts.of(-8);
  public static final Angle kMinBlockedAngle = Degrees.of(20); // this
  public static final Angle kMaxBlockedAngle = Degrees.of(40); // this

  // pivot homing
  public static final Voltage kHomingVoltage = Volts.of(-2);
  public static final Current kHomingCurrentThreshold = Amps.of(25);
  public static final AngularVelocity kHomingVelocityThreshold = DegreesPerSecond.of(2);

  // physical constants
  public static final double kPivotGearing = 150; // this
  public static final Distance kPivotLength = Inches.of(0.6); // this
  public static final Angle kPivotMinAngle = Degrees.of(0);
  public static final Angle kPivotMaxAngle = Degrees.of(180);
  public static final Angle kPivotStartingAngle = Degrees.of(0);
  public static final double kPivotMOI = 0.1; // this
  public static final Angle kPivotIntakeAngle =
      Degrees.of(80); // TODO: dummy angles; test these angles
  public static final Angle kPivotOuttakeAngle = Degrees.of(90);
  public static final Angle kPivotStoreAngle =
      Degrees.of(70); // the angle the pivot will be at by default
  public static final Angle kPivotClimbAngle = Degrees.of(45);
  public static final Angle kPivotFloorAngle = Degrees.of(0);
  public static final Voltage kPivotClimbVoltage = Volts.of(-8);

  // motor configurations
  public static final boolean kLeftInverted = false;
  public static final boolean kRightInverted = false;
  public static final int kSmartCurrentLimit = 40;
  public static final double kPivotPositionConversionFactor = 360 / kPivotGearing;
  public static final double kPivotVelocityConversionFactor = kPivotPositionConversionFactor / 60;
  public static final double kPivotVelocityConversionFactor = 360 / kPivotGearing;
  public static final double kPivotPositionConversionFactor = kPivotVelocityConversionFactor / 60;
  public static final Voltage kNominalVoltage = Volts.of(12);
}
