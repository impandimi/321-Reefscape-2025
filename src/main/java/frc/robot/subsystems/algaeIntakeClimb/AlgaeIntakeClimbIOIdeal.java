package frc.robot.subsystems.algaeIntakeClimb;

import edu.wpi.first.units.measure.Voltage;

public class AlgaeIntakeClimbIOIdeal implements AlgaeIntakeClimbIO{

    public static final AlgaeIntakeClimbConfig config = new AlgaeIntakeClimbConfig(0,0,0,0);

    public void updateInputs(AlgaeIntakeClimbInputs inputs){
        inputs.velocity = 0;
        inputs.currentAngle = 0;
        inputs.hasAlgae = false;
        inputs.rollerVelocity = 0;
        inputs.rollerAngle = 0;

    }
public void setPivotPower(Voltage volts){}
public void setRollerPower(Voltage volts){}

}
