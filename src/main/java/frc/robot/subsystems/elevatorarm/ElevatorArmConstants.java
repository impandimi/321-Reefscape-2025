/* (C) Robolancers 2025 */
package frc.robot.subsystems.elevatorarm;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;

public class ElevatorArmConstants {
  public static final int kElevatorArmId = 0;

  public static final double kElevatorArmGearing = 100;
  public static final double kElevatorArmMOI = 3;
  public static final Distance kElevatorArmLength = Meters.of(0.5);
  public static final Angle kMaxAngle = Degrees.of(360);
  public static final Angle kMinAngle = Degrees.of(0);

  public static final double kPositionConversionFactor = 360 / kElevatorArmGearing;
  public static final double kVelocityConversionFactor = kPositionConversionFactor / 60;

  public static final double kAbsPositionConversionFactor = 360;
  public static final double kAbsVelocityConversionFactor = kPositionConversionFactor / 60;

  public static final int kCurrentLimit = 40;
  public static final double kNominalVoltage = 12;
  public static final boolean kInverted = false;
}
