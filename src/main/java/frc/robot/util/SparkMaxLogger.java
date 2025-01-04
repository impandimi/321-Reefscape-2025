package frc.robot.util;

import com.revrobotics.spark.SparkMax;

import edu.wpi.first.epilogue.CustomLoggerFor;
import edu.wpi.first.epilogue.Epilogue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.logging.ClassSpecificLogger;
import edu.wpi.first.epilogue.logging.EpilogueBackend;

@CustomLoggerFor(SparkMax.class)
public class SparkMaxLogger extends ClassSpecificLogger<SparkMax> {

    public SparkMaxLogger() {
        super(SparkMax.class);
      }

    @Override
    protected void update(EpilogueBackend dataLogger, SparkMax object) {
        if (Epilogue.shouldLog(Logged.Importance.DEBUG)) {
            dataLogger.log("CAN ID", object.getDeviceId());
            dataLogger.log("Requested Speed (Duty Cycle)", object.get());
            dataLogger.log("Motor Voltage (V)", object.getAppliedOutput());
            dataLogger.log("Output Current (A)", object.getOutputCurrent());
            dataLogger.log("Motor Temperature (C)", object.getMotorTemperature());
            dataLogger.log("Bus Voltage (V)", object.getBusVoltage());
            dataLogger.log("Encoder Position", object.getEncoder().getPosition());
            dataLogger.log("Encoder Velocity", object.getEncoder().getVelocity());
        }
    }
    
}