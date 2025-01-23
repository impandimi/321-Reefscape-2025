/* (C) Robolancers 2025 */
package frc.robot.subsystems.coralendeffector;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.measure.Voltage;
import static edu.wpi.first.units.Units.Volts;

@Logged 
public class CoralEndEffectorIOReal implements CoralEndEffectorIO {

  private SparkMax motor;
  private DigitalInput touchSensor;
  private DigitalInput beamBreak;

  //   private AbsoluteEncoder absoluteEncoder = motor.getAbsoluteEncoder();

  public CoralEndEffectorIOReal() {
    this.motor = new SparkMax(CoralEndEffectorConstants.kMotorPort, MotorType.kBrushless);
    //motor.configure(
    //     // new SparkMaxConfig()
    //         // .inverted(CoralEndEffectorConstants.kInvertedMotor)
    //         // .smartCurrentLimit(CoralEndEffectorConstants.kCurrentLimit),
    //    //  null,
    //    // null);
    
        //new SparkBase.ResetMode() 
         //.inverted(CoralEndEffectorConstants.kInvertedMotor)
        // .smartCurrentLimit(CoralEndEffectorConstants.kCurrentLimit),
         //null,
         //null);

    //motor.configure(
   //     new SparkBase.PersistMode() 
      //   .inverted(CoralEndEffectorConstants.kInvertedMotor)
        // .smartCurrentLimit(CoralEndEffectorConstants.kCurrentLimit),
         //null,
         //null);
         

        
    touchSensor = new DigitalInput(CoralEndEffectorConstants.kTouchSensorPort);
    beamBreak = new DigitalInput(CoralEndEffectorConstants.kBeamBreakPort);
  }

  @Override
  public void setVoltage(Voltage volts) {
    motor.setVoltage(volts);
  }
  
  @Override
  public void updateInputs(CoralEndEffectorInputs inputs) {
    inputs.voltage = Volts.of(motor.getBusVoltage());
    inputs.hasCoral = touchSensor.get();
    inputs.isBeamBreakBroken = beamBreak.get();
  }
}
