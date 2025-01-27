package frc.robot.subsystems.algaeIntakeClimb;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.Voltage;

public class AlgaeIntakeClimbConstants {

    public static final double kintakeAngle = 0;
    public static final Voltage kintakePower = Volts.of(3);
    public static final Voltage kouttakePower = Volts.of(-3);
    public static final double kouttakeAngle = 90;
    public static final Voltage kclimbPower = Volts.of(2);
    public static final double kclimbAngle = 0;
    public static final Voltage kunclimbPower = Volts.of(-0.1);
    // TODO: all values are placeholders
    public static final boolean kinversionfactor = false;
    public static final int ksmartCurrentLimit = 0;
    public static final double kvelocityConversionFactor = 90;
    public static final double kpositionConversionFactor = 0;
    public static final double kclimbMotorPosition = 0;
    public static final Voltage khangPower = null;
    public static final double kfloorAngle = 180;
    public static final double kSimJKgSquaredMeters = 0;
    public static final double kSimGearing = 0;
    public static final double kSimArmLengthMeters = 0;
    public static final double kSimMinAngleRads = 0;
    public static final double kSimMaxAngleRads = 0;
    public static final double kSimStartingAngleRads = 0;
    public static final double kBeamBreakSimAngle = 0;

    //TODO: also need the values for:
    // pid/feedforward config(tuning)
    // motor and sensor device id
    // 
}
