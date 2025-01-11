/* (C) Robolancers 2025 */
import edu.wpi.first.wpilibj.RobotController;

public class Constants {

  public static final class AlgaeIntakeConstants {

    public static final int kMotorPort = 0;
    public static final boolean kInvertMotor = false;
    public static final int kCurrentLimit = 40;
    public static final double kIntakeVoltage = 0.25 * RobotController.getBatteryVoltage();
  }

  public static final class AlgaeRetractorConstants {

    public static final int kMotorPort = 0;
  }

  public static final class CoralIntakeConstants {

    public static final int kMotorPort = 0;
  }

  public static final class CoralRetractorConstants {

    public static final int kMotorPort = 0;
  }
}
