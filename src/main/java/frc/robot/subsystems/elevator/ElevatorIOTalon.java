/* (C) Robolancers 2025 */
package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Volt;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;

@Logged
// For when Elevator is real
public class ElevatorIOTalon implements ElevatorIO {
  // Creates config record w/ values
  public static final ElevatorConfig config = new ElevatorConfig(50, 0, 0, 0, 0, 0.25, 0.01);

  // Creates motor objects

  public TalonFX elevatorMotorLeft = new TalonFX(ElevatorConstants.kLeftMotorID);

  // NOTE: Right motor MAY be commented out in order to test one motor

  public TalonFX elevatorMotorRight = new TalonFX(ElevatorConstants.kRightMotorID);

  // Constructor: Sets up motors
  public ElevatorIOTalon() {
    setupMotors();
  }

  // Updates inputs with values from encoder (Called periodically in periodic function later)
  public void updateInputs(ElevatorInputs inputs) {
    inputs.height =
        Meters.of(
            elevatorMotorRight.getPosition().getValueAsDouble()
                * ElevatorConstants.kElevatorConversion.in(Meters));
    inputs.velocity =
        MetersPerSecond.of(
            elevatorMotorRight.getVelocity().getValueAsDouble()
                * ElevatorConstants.kElevatorConversion.in(Meters));
    inputs.current = Amps.of(elevatorMotorRight.getStatorCurrent().getValueAsDouble());
  }

  // Method to setup L & R motor & encoders
  // NOTE: Right motor follows left & only left motor encoder is used
  private void setupMotors() {
    TalonFXConfiguration configurationLeft =
        new TalonFXConfiguration()
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withStatorCurrentLimit(ElevatorConstants.kCurrentLimit)
                    .withStatorCurrentLimitEnable(true))
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withInverted(
                        ElevatorConstants.kLeftInverted
                            ? InvertedValue.Clockwise_Positive
                            : InvertedValue.CounterClockwise_Positive)
                    .withNeutralMode(NeutralModeValue.Brake))
            .withFeedback(
                new FeedbackConfigs()
                    .withSensorToMechanismRatio(ElevatorConstants.kElevatorGearing)
                    .withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor));

    TalonFXConfiguration configurationRight =
        new TalonFXConfiguration()
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withStatorCurrentLimit(ElevatorConstants.kCurrentLimit)
                    .withStatorCurrentLimitEnable(true))
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withInverted(
                        ElevatorConstants.kRightInverted
                            ? InvertedValue.Clockwise_Positive
                            : InvertedValue.CounterClockwise_Positive)
                    .withNeutralMode(NeutralModeValue.Brake))
            .withFeedback(
                new FeedbackConfigs()
                    .withSensorToMechanismRatio(ElevatorConstants.kElevatorGearing)
                    .withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor))
            .withMotionMagic(
                new MotionMagicConfigs()
                    .withMotionMagicCruiseVelocity(
                        MetersPerSecond.of(0).in(MetersPerSecond)
                            / ElevatorConstants.kElevatorConversion.in(Meters))
                    .withMotionMagicAcceleration(
                        MetersPerSecondPerSecond.of(120).in(MetersPerSecondPerSecond)
                            / ElevatorConstants.kElevatorConversion.in(Meters)))
            .withSlot0(
                new Slot0Configs()
                    .withGravityType(GravityTypeValue.Elevator_Static)
                    .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign));

    elevatorMotorLeft.getConfigurator().apply(configurationLeft);
    elevatorMotorRight.getConfigurator().apply(configurationRight);
  }

  // Sets power of motors w/voltage
  public void setVoltage(Voltage Volts) {
    elevatorMotorRight.setVoltage(Volts.in(Volt));
    elevatorMotorLeft.setControl(new Follower(elevatorMotorRight.getDeviceID(), true));
  }

  @Override
  public void setPosition(Distance position) {
    System.out.println(position.in(Meters) / ElevatorConstants.kElevatorConversion.in(Meters));
    elevatorMotorRight.setControl(
        new MotionMagicExpoVoltage(
            position.in(Meters) / ElevatorConstants.kElevatorConversion.in(Meters)));
    elevatorMotorLeft.setControl(new Follower(elevatorMotorRight.getDeviceID(), true));
  }

  // Sets encoder pos
  public void setEncoderPosition(Distance position) {
    elevatorMotorLeft.setPosition(
        position.in(Meters) / ElevatorConstants.kElevatorConversion.in(Meters));
    elevatorMotorRight.setPosition(
        position.in(Meters) / ElevatorConstants.kElevatorConversion.in(Meters));
  }

  // Special case where encoder pos is reset to the initial/starting height
  public void resetEncoderPosition() {
    setEncoderPosition(ElevatorConstants.kElevatorStartingHeight);
  }

  @Override
  public void setOnboardPID(ElevatorConfig conf) {
    elevatorMotorRight
        .getConfigurator()
        .apply(
            new Slot0Configs()
                .withGravityType(GravityTypeValue.Elevator_Static)
                .withKP(conf.kP())
                .withKI(conf.kI())
                .withKD(conf.kD())
                .withKS(conf.kS())
                .withKG(conf.kG())
                .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign)
                .withKV(conf.kV())
                .withKA(conf.kA()));
  }
}
