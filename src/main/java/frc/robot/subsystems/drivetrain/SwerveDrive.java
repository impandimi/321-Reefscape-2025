/* (C) Robolancers 2025 */
package frc.robot.subsystems.drivetrain;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.util.DriveFeedforwards;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

/*
 * drive interface. A real and sim implementation is made out of this. Using this, so we can implement maplesim
 */
@Logged
public interface SwerveDrive extends Subsystem {

  PIDController xPoseController = new PIDController(0, 0, 0);
  PIDController yPoseController = new PIDController(0, 0, 0);
  PIDController thetaController = new PIDController(0, 0, 0);

  public default void configureAutoBuilder() {
    try { // try and catch for config exception
      var config = RobotConfig.fromGUISettings();
      AutoBuilder.configure(
          this::getPose, // Supplier of current robot pose
          this::resetPose, // Consumer for seeding pose against auto
          this::getChassisSpeeds, // Supplier of current robot speeds
          // Consumer of ChassisSpeeds and feedforwards to drive the robot
          (speeds, feedforwards) ->
              driveRobotCentric(
                  () -> speeds.vxMetersPerSecond,
                  () -> speeds.vyMetersPerSecond,
                  () -> speeds.omegaRadiansPerSecond,
                  feedforwards),
          new PPHolonomicDriveController(
              // PID constants for translation
              new PIDConstants(
                  DrivetrainConstants.kTranslationGains.kP(),
                  DrivetrainConstants.kTranslationGains.kI(),
                  DrivetrainConstants.kTranslationGains.kD()),
              // PID constants for rotation
              new PIDConstants(
                  DrivetrainConstants.kHeadingGains.kP(),
                  DrivetrainConstants.kHeadingGains.kI(),
                  DrivetrainConstants.kHeadingGains.kD())),
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

  // configure driveToPoseControllers
  default void configurePoseControllers() {
    xPoseController.setPID(
        DrivetrainConstants.kTranslationGains.kP(),
        DrivetrainConstants.kTranslationGains.kI(),
        DrivetrainConstants.kTranslationGains.kD());

    yPoseController.setPID(
        DrivetrainConstants.kTranslationGains.kP(),
        DrivetrainConstants.kTranslationGains.kI(),
        DrivetrainConstants.kTranslationGains.kD());

    thetaController.setPID(
        DrivetrainConstants.kHeadingGains.kP(),
        DrivetrainConstants.kHeadingGains.kI(),
        DrivetrainConstants.kHeadingGains.kD());

    thetaController.enableContinuousInput(-Math.PI, Math.PI);
  }

  Command driveFieldCentric(
      DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier rotation);

  Command driveRobotCentric(
      DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier rotation);

  Command driveRobotCentric(
      DoubleSupplier translationX,
      DoubleSupplier translationY,
      DoubleSupplier rotation,
      DriveFeedforwards feedforwards);

  Command driveFixedHeading(
      DoubleSupplier translationX, DoubleSupplier translationY, Supplier<Rotation2d> rotation);

  // auto drive w/ external pid controllers
  Command driveToPose(Pose2d pose);

  void resetPose(Pose2d pose);

  void setSwerveModuleStates(SwerveModuleState[] states);

  SwerveModuleState[] getMeasuredModuleStates();

  SwerveModuleState[] getTargetModuleStates();

  Pose2d getPose();

  ChassisSpeeds getChassisSpeeds();

  Rotation2d getHeading();

  void addVisionMeasurement(Pose2d visionRobotPose, double timeStampSeconds);
}
