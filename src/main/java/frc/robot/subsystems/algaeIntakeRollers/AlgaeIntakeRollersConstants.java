/* (C) Robolancers 2025 */
package frc.robot.subsystems.algaeIntakerollers;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.measure.Voltage;

@Logged
public class AlgaeIntakeRollersConstants {

  // motor, sensor IDs
  public static final int kBeamBreakId = 0;
  public static final int kMotorId = 0;

  // function constants
  public static final Voltage kRollerIntakeVoltage = Volts.of(8); // test
  public static final Voltage kRollerOuttakeVoltage = Volts.of(-5); // test

  // motor configurations
  public static final boolean kRollerInverted = false;
  public static final int kSmartCurrentLimit = 40;
  public static final double kRollerVelocityConversionFactor = 2; // ask mech
  public static final double kRollerPositionConversionFactor = 120; // ask mech
  public static final Voltage kNominalVoltage = Volts.of(12);

  // physical constants
  public static final double kRollerGearing = 1; // ask mech
  public static final double kRollerMOI = 0.01; // ask mech
}
