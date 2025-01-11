/* (C) Robolancers 2025 */
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

public class AlgaeIntakeIOReal implements AlgaeIntakeIO {

  private SparkMax motor;

  public AlgaeIntakeIOReal() {
    motor = new SparkMax(AlgaeIntakeConstants.kMotorPort, MotorType.kBrushless);
    motor.configure(
        new SparkMaxConfig()
            .inverted(AlgaeIntakeConstants.kInvertMotor)
            .smartCurrentLimit(AlgaeIntakeConstants.kCurrentLimit),
        null,
        null);
  }

  @Override
  public void updateInputs(AlgaeIntakeInputs inputs) {
    inputs.power = motor.get();
  }

  @Override
  public void setVoltage(double voltage) {
    motor.setVoltage(voltage);
  }
}
