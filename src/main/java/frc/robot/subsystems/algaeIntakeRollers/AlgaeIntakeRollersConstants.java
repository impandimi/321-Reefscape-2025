/* (C) Robolancers 2025 */
package frc.robot.subsystems.algaeIntakeRollers;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.Voltage;

public class AlgaeIntakeRollersConstants {

  // function constants
  public static final Voltage kRollerIntakeVoltage = Volts.of(3); // test
  public static final Voltage kRollerOuttakeVoltage = Volts.of(-3); // test

  // motor configurations
  public static final boolean kRollerInverted = false;
  public static final int kSmartCurrentLimit = 40;
  public static final double kRollerVelocityConversionFactor = 0; // ask mech
  public static final double kRollerPositionConversionFactor = 0; // ask mech
  public static final Voltage kNominalVoltage = Volts.of(12);

  // physical constants
  public static final double kRollerGearing = 0; // ask mech
  public static final double kRollerMOI = 0; // ask mech

  // motor, sensor IDs
  public static final int kDigitalInputID = 0;
  public static final int kMotorID = 0;

}
