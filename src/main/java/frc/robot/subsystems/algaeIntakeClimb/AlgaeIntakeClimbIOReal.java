package frc.robot.subsystems.algaeIntakeClimb;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;

import com.revrobotics.spark.SparkLowLevel.MotorType;

public class AlgaeIntakeClimbIOReal implements AlgaeIntakeClimbIO{

    public static final AlgaeIntakeClimbConfig config = new AlgaeIntakeClimbConfig(1,2,3,4);

//device ids are plcaeholders
private SparkMax pivotMotorLeft = new SparkMax(1,MotorType.kBrushless);
AbsoluteEncoder AlgaeIntakeClimbEncoder = pivotMotorLeft.getAbsoluteEncoder();

private SparkMax rollerMotorLeft = new SparkMax(3,MotorType.kBrushless);


public AlgaeIntakeClimbIOReal(){
    configureMotors();
}

public void configureMotors(){
pivotMotorLeft.configure(
    new SparkMaxConfig()
    .inverted(AlgaeIntakeClimbConstants.kinversionfactor)
    .smartCurrentLimit(AlgaeIntakeClimbConstants.ksmartCurrentLimit)
        .apply(new EncoderConfig()
        .velocityConversionFactor(AlgaeIntakeClimbConstants.kvelocityConversionFactor)
        .positionConversionFactor(AlgaeIntakeClimbConstants.kpositionConversionFactor)),
  ResetMode.kResetSafeParameters,
  PersistMode.kPersistParameters);

  rollerMotorLeft.configure(
    new SparkMaxConfig()
    .inverted(AlgaeIntakeClimbConstants.kinversionfactor)
    .smartCurrentLimit(AlgaeIntakeClimbConstants.ksmartCurrentLimit),
  ResetMode.kResetSafeParameters,
  PersistMode.kPersistParameters);

}

DigitalInput algaeSensor = new DigitalInput(1);

public void setPivotPower(Voltage volts){
    pivotMotorLeft.setVoltage(volts);
}

public void setRollerPower(Voltage volts){
    rollerMotorLeft.setVoltage(volts);
}

public void updateInputs(AlgaeIntakeClimbInputs inputs){
    inputs.velocity = AlgaeIntakeClimbEncoder.getVelocity();
    inputs.currentAngle = AlgaeIntakeClimbEncoder.getPosition();
    inputs.hasAlgae = algaeSensor.get();
    inputs.rollerVelocity = rollerMotorLeft.getEncoder().getVelocity();
    inputs.rollerAngle = rollerMotorLeft.getEncoder().getPosition();
}
}
