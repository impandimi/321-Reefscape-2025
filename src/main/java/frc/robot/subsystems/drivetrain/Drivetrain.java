/* (C) Robolancers 2025 */
package frc.robot.subsystems.drivetrain;

import static edu.wpi.first.units.Units.MetersPerSecond;

import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.util.MathUtils;

@Logged
public class Drivetrain extends TunerSwerveDrivetrain implements Subsystem {
  public record Gains(double kP, double kI, double kD) {}

  public static final Gains kPPDrive = new Gains(5, 0, 0);
  public static final Gains kPPTurn = new Gains(5, 0, 0);

  public Drivetrain create() {
    return RobotBase.isReal()
        ? new Drivetrain(
            TunerConstants.DrivetrainConstants,
            TunerConstants.FrontLeft,
            TunerConstants.FrontRight,
            TunerConstants.BackLeft,
            TunerConstants.BackRight)
        : new Drivetrain(null, null, null);
  }

  public Drivetrain(
      SwerveDrivetrainConstants drivetrainConstants, SwerveModuleConstants<?, ?, ?>... modules) {
    super(drivetrainConstants, modules);
  }

  public void configureAutoBuilder() {
    try {
      var config = RobotConfig.fromGUISettings();
      AutoBuilder.configure(
          () -> getState().Pose, // Supplier of current robot pose
          this::resetPose, // Consumer for seeding pose against auto
          () -> getState().Speeds, // Supplier of current robot speeds
          // Consumer of ChassisSpeeds and feedforwards to drive the robot
          (speeds, feedforwards) ->
              setControl(
                  new SwerveRequest.ApplyRobotSpeeds()
                      .withWheelForceFeedforwardsX(feedforwards.robotRelativeForcesXNewtons())
                      .withWheelForceFeedforwardsY(feedforwards.robotRelativeForcesYNewtons())),
          new PPHolonomicDriveController(
              // PID constants for translation
              new PIDConstants(kPPDrive.kP(), kPPDrive.kI(), kPPDrive.kD()),
              // PID constants for rotation
              new PIDConstants(kPPTurn.kP(), kPPTurn.kI(), kPPTurn.kD())),
          config,
          // Assume the path needs to be flipped for Red vs Blue, this is normally the case
          () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
          this // Subsystem for requirements
          );
    } catch (Exception ex) {
      DriverStation.reportError(
          "Failed to load PathPlanner config and configure AutoBuilder", ex.getStackTrace());
    }
  }

  public Command driveFieldCentric(double translationX, double translationY, double rotation) {
    return run(
        () -> {
          this.setControl(
              new SwerveRequest.FieldCentric()
                  .withDriveRequestType(DriveRequestType.Velocity)
                  .withDeadband(DrivetrainConstants.kDriveDeadband)
                  .withRotationalDeadband(DrivetrainConstants.kRotationDeadband)
                  .withVelocityX(translationX * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond))
                  .withVelocityY(translationY * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond))
                  .withRotationalRate(
                      MathUtils.squareKeepSign(rotation)
                          * DrivetrainConstants.kMaxAngularVelocity));
        });
  }

  @Override
  public void simulationPeriodic() {
    this.updateSimState(0.02, RobotController.getBatteryVoltage());
  }
}
