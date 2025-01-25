package frc.robot.subsystems.algaelow;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.units.measure.Voltage;

import com.revrobotics.spark.SparkLowLevel.MotorType;

public class AlgaelowIOReal implements AlgaelowIO{

    public static final AlgaelowConfig config = new AlgaelowConfig(1,2,3,4);

private SparkMax pivotMotorLeft = new SparkMax(1,MotorType.kBrushless);
private SparkMax pivotMotorRight = new SparkMax(2,MotorType.kBrushless);
AbsoluteEncoder AlgaelowEncoder = pivotMotorLeft.getAbsoluteEncoder();


private SparkMax rollerMotorLeft = new SparkMax(3,MotorType.kBrushless);
private SparkMax rollerMotorRight = new SparkMax(4,MotorType.kBrushless);


//configure motors

//beam break + configure

public void setPivotPower(Voltage volts){
    pivotMotorLeft.setVoltage(volts);
    pivotMotorRight.setVoltage(volts);
}

public void setRollerPower(Voltage volts){
    rollerMotorLeft.setVoltage(volts);
    rollerMotorRight.setVoltage(volts);
}


public void updateInputs(AlgaelowInputs inputs){
    inputs.velocity = AlgaelowEncoder.getVelocity();
    inputs.currentAngle = AlgaelowEncoder.getPosition();
    //sensors for boolean inputs
}

}

