/* (C) Robolancers 2025 */
package frc.robot.subsystems.drivetrain;

import static edu.wpi.first.units.Units.MetersPerSecond;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.util.MathUtils;

@Logged

/*
 * Drivetrain based on CTR SwerveDrive
 */

public class Drivetrain extends SwerveDrivetrain<TalonFX, TalonFX, CANcoder> implements Subsystem {
  // auto gains
  public record Gains(double kP, double kI, double kD) {}

  public static final Gains kPPDrive = new Gains(5, 0, 0);
  public static final Gains kPPTurn = new Gains(5, 0, 0);

  // no sim b/c sim runs in simulationPeriodic
  public Drivetrain create() {
    return RobotBase.isReal()
        ? new Drivetrain(
            TunerConstants.DrivetrainConstants,
            TunerConstants.FrontLeft,
            TunerConstants.FrontRight,
            TunerConstants.BackLeft,
            TunerConstants.BackRight)
        : new DrivetrainSim(
            TunerConstants.DrivetrainConstants,
            TunerConstants.FrontLeft,
            TunerConstants.FrontRight,
            TunerConstants.BackLeft,
            TunerConstants.BackRight);
  }

  public Drivetrain(
      SwerveDrivetrainConstants drivetrainConstants, SwerveModuleConstants<?, ?, ?>... modules) {
    super(TalonFX::new, TalonFX::new, CANcoder::new, drivetrainConstants, modules);
    configureAutoBuilder();
  }

  public void configureAutoBuilder() {
    try { // try and catch for config exception
      var config = RobotConfig.fromGUISettings();
      AutoBuilder.configure(
          this::getPose, // Supplier of current robot pose
          this::resetPose, // Consumer for seeding pose against auto
          this::getChassisSpeeds, // Supplier of current robot speeds
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

  // default drive command
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

  public Pose2d getPose() {
    return this.getState().Pose;
  }

  public ChassisSpeeds getChassisSpeeds() {
    return this.getState().Speeds;
  }
}
