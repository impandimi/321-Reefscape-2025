/* (C) Robolancers 2025 */
package frc.robot.subsystems.coralendeffector;

import edu.wpi.first.wpilibj.RobotController;

public class CoralEndEffectorConstants {

  public static final int kMotorPort = 0;
  public static final boolean kInvertedMotor = false;
  public static final int kCurrentLimit = 40;
  public static final double kIntakeVoltage = 0.25 * RobotController.getBatteryVoltage();
  public static final double kOuttakeVoltage = -0.25 * RobotController.getBatteryVoltage();
  public static final double kStallVoltage = 0.1 * RobotController.getBatteryVoltage();

  public static final int kBeamBreakPort = 0;

  public static final int kTouchSensorPort = 0;
}
