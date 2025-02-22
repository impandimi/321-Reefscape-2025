/* (C) Robolancers 2025 */
package frc.robot.subsystems.algaeIntakeRollers;

import static edu.wpi.first.units.Units.RPM;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;

@Logged
public class AlgaeIntakeRollersIOKraken implements AlgaeIntakeRollersIO {
  // kraken implementation of real mechanism
  public static final AlgaeIntakeRollersConfig config = new AlgaeIntakeRollersConfig(0, 0, 0, 0);

  private DigitalInput algaeSensor = new DigitalInput(AlgaeIntakeRollersConstants.kBeamBreakId);

  TalonFX rollerMotorLeft =
      new TalonFX(AlgaeIntakeRollersConstants.kMotorId); 
    
  VoltageOut voltageRequest = new VoltageOut(0); 

  public AlgaeIntakeRollersIOKraken() {
    rollerMotorLeft // sets up and creates left pivot motor
        .getConfigurator()
        .apply(
            new CurrentLimitsConfigs()
                .withStatorCurrentLimitEnable(true)
                .withStatorCurrentLimit(AlgaeIntakeRollersConstants.kSmartCurrentLimit));
    rollerMotorLeft
        .getConfigurator()
        .apply(
            new MotorOutputConfigs()
                .withNeutralMode(NeutralModeValue.Brake)
                .withInverted(
                    AlgaeIntakeRollersConstants.kRollerInverted
                        ? InvertedValue.CounterClockwise_Positive
                        : InvertedValue.Clockwise_Positive));
    rollerMotorLeft
        .getConfigurator()
        .apply(
            new FeedbackConfigs()
                .withSensorToMechanismRatio(AlgaeIntakeRollersConstants.kRollerGearing));
  }

  public void setRollerVoltage(Voltage volts) {
    rollerMotorLeft.setControl(
        voltageRequest.withOutput(volts)); 
  }

  public void updateInputs(AlgaeIntakeRollersInputs inputs) { 
    inputs.hasAlgae = algaeSensor.get();
    inputs.rollerVelocity = rollerMotorLeft.getVelocity().getValue();
  }

  }

