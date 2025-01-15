package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;

public class ElevatorConstants {
    public static final Distance kstationHeight = Meters.of(0); // TODO: tune height
    public static final Distance kL1Height = Meters.of(0);
    public static final Distance kL2Height = Meters.of(0); 
    public static final Distance kL3Height = Meters.of(0); 
    public static final Distance kL4Height = Meters.of(0); 
    public static final Distance kTravelHeight = Meters.of(0);
    public static final double kElevatorGearing = Math.PI;
    public static final double kBrake = 1;
    public static final double kVelocityConversionFactor = 1000;
    public static final double kPositionConversionFactor = 1000;
    public static final double kCarriageMass = 1;
    public static final double kElevatorDrumRadius = 1;
    public static final double kMaxElevatorHeightMeters = 5;
    public static final double kMinElevatorHeightMeters = 0;
    public static final Voltage kVoltageHoming = Volts.of(-1);
}
