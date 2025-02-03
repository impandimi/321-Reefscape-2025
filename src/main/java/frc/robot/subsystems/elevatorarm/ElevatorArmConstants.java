/* (C) Robolancers 2025 */
package frc.robot.subsystems.elevatorarm;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;

public class ElevatorArmConstants {
  // the CAN ID of the arm motor on the elevator
  public static final int kElevatorArmId = 0;
  public static final int kEncoderCANdiId = 0;

  // the gearing of the arm
  public static final double kElevatorArmGearing = 100;
  // conversion factors for the relative encoder into angle in degrees
  public static final double kPositionConversionFactor = 360 / kElevatorArmGearing;
  public static final double kVelocityConversionFactor = kPositionConversionFactor / 60;

  // conversion factors for the absolute encoder into angle in degrees
  public static final double kAbsPositionConversionFactor = 360;
  public static final double kAbsVelocityConversionFactor = kPositionConversionFactor / 60;

  // motor characteristics
  public static final int kCurrentLimit = 40;
  public static final double kNominalVoltage = 12;
  public static final boolean kInverted = false;

  // absolute encoder port on RoboRIO DIO if we need it
  public static final int kAbsoluteEncoderPort = 0;

  // absolute encoder offset
  public static final Angle kAbsoluteEncoderOffset = Degrees.of(0);

  // **** For simulation use ****
  // the MOI of the arm
  public static final double kElevatorArmMOI = 3;
  // the length of the arm
  public static final Distance kElevatorArmLength = Meters.of(0.5);
  // the maximum angle the arm can go in simulation
  public static final Angle kMaxAngle = Degrees.of(360);
  // the minimum angle the arm can go in simulation
  public static final Angle kMinAngle = Degrees.of(0);
}
