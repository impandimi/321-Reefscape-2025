package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.units.measure.Voltage;

import com.revrobotics.spark.SparkLowLevel.MotorType;

public class ElevatorIOReal implements ElevatorIO {

    public static final ElevatorConfig config = new ElevatorConfig(0,0,0,0);

    private SparkMax elevatorMotor = new SparkMax(0, MotorType.kBrushless);
    private RelativeEncoder motorEncoder = elevatorMotor.getEncoder();

public ElevatorIOReal(){
    setUp();
}
public void setUp (){
    SparkMaxConfig config = new SparkMaxConfig();

config
    .inverted(true)
    .idleMode(IdleMode.kBrake);
config.encoder
    .positionConversionFactor(ElevatorConstants.kPositionConversionFactor)
    .velocityConversionFactor(1000);
config.closedLoop
    .feedbackSensor(FeedbackSensor.kPrimaryEncoder);
}

public void updateInputs(ElevatorInputs inputs){
        inputs.height = Meters.of(motorEncoder.getPosition());
        inputs.velocity = MetersPerSecond.of(motorEncoder.getVelocity());
        inputs.current = Amps.of(elevatorMotor.getOutputCurrent());
    }

public void setPower(Voltage volts){
    elevatorMotor.setVoltage(volts);  
}

public void resetEncoder(){
    motorEncoder.setPosition(0);
}
}

