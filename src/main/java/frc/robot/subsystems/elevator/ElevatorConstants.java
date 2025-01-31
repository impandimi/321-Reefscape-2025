/* (C) Robolancers 2025 */
package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Pound;

import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;

public class ElevatorConstants {

  // List of Elevator specific constants that
  public static final int kLeftMotorID = 12;
  public static final int kRightMotorID = 0;
  public static final InvertedValue kInverted = InvertedValue.Clockwise_Positive;
  public static final int kCurrentLimit = 40;
  public static final double kElevatorGearing = 20;
  public static final Distance kElevatorConversion = Inches.of(0.375 * 2 * 13);
  public static final double kPositionConversionFactor =
      kElevatorConversion.in(Meters) / kElevatorGearing;
  public static final double kVelocityConversionFactor = kPositionConversionFactor / 60;
  public static final Mass kElevatorCarriageMass = Pound.of(20);
  public static final Distance kElevatorDrumRadius = kElevatorConversion.div(2 * Math.PI);
  // TODO:I bet if we measure we can get a more accurate minimum height :)
  public static final Distance kElevatorMinimumHeight = Inches.of(27);
  public static final Distance kElevatorMaximumHeight = Inches.of(56);
  public static final Distance kElevatorStartingHeight = kElevatorMinimumHeight;
}
