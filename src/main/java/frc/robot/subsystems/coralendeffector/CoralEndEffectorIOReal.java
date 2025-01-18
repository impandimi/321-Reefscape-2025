/* (C) Robolancers 2025 */
package frc.robot.subsystems.coralendeffector;

// import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.DigitalInput;

public class CoralEndEffectorIOReal implements CoralEndEffectorIO {

  private SparkMax motor;
  private DigitalInput touchSensor;
  private DigitalInput beamBreak;

  //   private AbsoluteEncoder absoluteEncoder = motor.getAbsoluteEncoder();

  public CoralEndEffectorIOReal() {
    motor = new SparkMax(CoralEndEffectorConstants.kMotorPort, MotorType.kBrushless);
    motor.configure(
        new SparkMaxConfig()
            .inverted(CoralEndEffectorConstants.kInvertedMotor)
            .smartCurrentLimit(CoralEndEffectorConstants.kCurrentLimit),
        null,
        null);
    touchSensor = new DigitalInput(CoralEndEffectorConstants.kTouchSensorPort);
    beamBreak = new DigitalInput(CoralEndEffectorConstants.kBeamBreakPort);
  }

  @Override
  public void updateInputs(CoralEndEffectorInputs inputs) {
    inputs.power = motor.get();
    inputs.hasCoral = touchSensor.get();
    inputs.isBroken = beamBreak.get();
  }

  @Override
  public void setVoltage(double voltage) {
    if (beamBreak.get() == true) {
      motor.setVoltage(CoralEndEffectorConstants.kStallVoltage);
    } else {
      motor.setVoltage(voltage);
    }
  }
}
