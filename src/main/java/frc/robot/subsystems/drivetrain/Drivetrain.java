/* (C) Robolancers 2025 */
package frc.robot.subsystems.drivetrain;

import static edu.wpi.first.units.Units.Seconds;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.ModuleRequest;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.util.DriveFeedforwards;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

/*
 * Real Drivetrain using CTRE SwerveDrivetrain and SwerveRequests
 */

@Logged
public class Drivetrain extends SwerveDrivetrain<TalonFX, TalonFX, CANcoder>
    implements SwerveDrive {

  public Drivetrain(
      SwerveDrivetrainConstants drivetrainConstants, SwerveModuleConstants<?, ?, ?>... modules) {
    // create CTRE Swervedrivetrain
    super(TalonFX::new, TalonFX::new, CANcoder::new, drivetrainConstants, modules);
    configNeutralMode(NeutralModeValue.Brake);
    configureAutoBuilder();
    configurePoseControllers();
  }

  @Override
  public Command driveFieldCentric(
      DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier rotation) {
    return run(
        () -> {
          var speeds =
              ChassisSpeeds.discretize(
                  translationX.getAsDouble(),
                  translationY.getAsDouble(),
                  rotation.getAsDouble(),
                  DrivetrainConstants.kLoopDt.in(Seconds));

          // x braking
          // if(Math.abs(newTranslationX) < DriveConstants.kDriveDeadband &&
          // Math.abs(newTranslationY) < DriveConstants.kDriveDeadband &&
          // Math.abs(newRotation) < DriveConstants.kRotationDeadband){
          // setControl(new SwerveRequest.SwerveDriveBrake())};

          setControl(
              new SwerveRequest.FieldCentric()
                  .withDriveRequestType(DriveRequestType.Velocity)
                  .withVelocityX(speeds.vxMetersPerSecond)
                  .withVelocityY(speeds.vyMetersPerSecond)
                  .withRotationalRate(speeds.omegaRadiansPerSecond));
        });
  }
  ;

  @Override
  public Command driveRobotCentric(
      DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier rotation) {

    return driveRobotCentric(translationX, translationY, rotation, DriveFeedforwards.zeros(4));
  }

  @Override
  public Command driveRobotCentric(
      DoubleSupplier translationX,
      DoubleSupplier translationY,
      DoubleSupplier rotation,
      DriveFeedforwards feedforwards) {
    return run(
        () -> {
          var speeds =
              ChassisSpeeds.discretize(
                  translationX.getAsDouble(),
                  translationY.getAsDouble(),
                  rotation.getAsDouble(),
                  DrivetrainConstants.kLoopDt.in(Seconds));

          setControl(
              new SwerveRequest.ApplyRobotSpeeds()
                  .withSpeeds(speeds)
                  .withDriveRequestType(DriveRequestType.Velocity)
                  .withWheelForceFeedforwardsX(feedforwards.robotRelativeForcesXNewtons())
                  .withWheelForceFeedforwardsY(feedforwards.robotRelativeForcesYNewtons()));
        });
  }

  @Override
  public Command driveToPose(Pose2d pose) {
    return runOnce(
            () -> {
              xPoseController.reset();
              yPoseController.reset();
              thetaController.reset();
            })
        .andThen(
            run(
                () -> {
                  var targetSpeeds =
                      ChassisSpeeds.discretize(
                          xPoseController.calculate(getPose().getX(), pose.getX())
                              * DrivetrainConstants.kMaxLinearVelocity,
                          yPoseController.calculate(getPose().getY(), pose.getY())
                              * DrivetrainConstants.kMaxLinearVelocity,
                          thetaController.calculate(
                                  getPose().getRotation().getRadians(),
                                  pose.getRotation().getRadians())
                              * DrivetrainConstants.kMaxAngularVelocity,
                          DrivetrainConstants.kLoopDt.in(Seconds));

                  setControl(new SwerveRequest.ApplyRobotSpeeds().withSpeeds(targetSpeeds));
                }));
  }

  // drive with absolute heading control
  @Override
  public Command driveFixedHeading(
      DoubleSupplier translationX, DoubleSupplier translationY, Supplier<Rotation2d> rotation) {
    return run(
        () -> {
          var speeds =
              ChassisSpeeds.discretize(
                  translationX.getAsDouble(),
                  translationY.getAsDouble(),
                  0,
                  DrivetrainConstants.kLoopDt.in(Seconds));

          setControl(
              new SwerveRequest.FieldCentricFacingAngle()
                  .withDriveRequestType(DriveRequestType.Velocity)
                  .withVelocityX(speeds.vxMetersPerSecond)
                  .withVelocityY(speeds.vyMetersPerSecond)
                  .withTargetDirection(rotation.get()));
        });
  }

  @Override
  public void resetPose(Pose2d pose) {
    super.resetPose(pose);
  }

  @Override
  public void setSwerveModuleStates(SwerveModuleState[] states) {
    for (int i = 0; i < super.getModules().length; i++) {
      super.getModule(i).apply(new ModuleRequest().withState(states[i]));
    }
  }

  @Logged(name = "MeasuredSwerveStates")
  @Override
  public SwerveModuleState[] getMeasuredModuleStates() {
    return super.getState().ModuleStates;
  }

  @Logged(name = "TargetSwerveStates")
  @Override
  public SwerveModuleState[] getTargetModuleStates() {
    return super.getState().ModuleTargets;
  }

  @Override
  public Pose2d getPose() {
    return super.getState().Pose;
  }

  @Override
  public ChassisSpeeds getChassisSpeeds() {
    return super.getState().Speeds;
  }

  @Override
  public Rotation2d getHeading() {
    return Rotation2d.fromDegrees(super.getPigeon2().getYaw().getValueAsDouble());
  }

  @Override
  public void addVisionMeasurement(Pose2d visionRobotPoseMeters, double timestampSeconds) {
    super.addVisionMeasurement(visionRobotPoseMeters, timestampSeconds);
  }
}
