/* (C) Robolancers 2025 */
package frc.robot.subsystems.drivetrain;

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
    configureControllers();
  }

  @Override
  public Command driveAuto(ChassisSpeeds speeds, DriveFeedforwards feedforwards) {
    return run(
        () ->
            setControl(
                new SwerveRequest.ApplyRobotSpeeds()
                    .withSpeeds(speeds)
                    .withWheelForceFeedforwardsX(feedforwards.robotRelativeForcesXNewtons())
                    .withWheelForceFeedforwardsY(feedforwards.robotRelativeForcesYNewtons())));
  }

  @Override
  public Command driveFieldCentric(
      DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier rotation) {
    var speeds =
        ChassisSpeeds.discretize(
            calculateDriveVelocity(
                translationX.getAsDouble(), translationY.getAsDouble(), rotation.getAsDouble()),
            DrivetrainConstants.kLoopDt.magnitude());

    // x braking
    // if(Math.abs(newTranslationX) < DriveConstants.kDriveDeadband &&
    // Math.abs(newTranslationY) < DriveConstants.kDriveDeadband &&
    // Math.abs(newRotation) < DriveConstants.kRotationDeadband){

    // return run (
    //     () -> {
    //       io.setControl(new SwerveRequest.SwerveDriveBrake());
    //     }
    //   );
    // }

    return run(
        () ->
            setControl(
                new SwerveRequest.FieldCentric()
                    .withDriveRequestType(DriveRequestType.Velocity)
                    .withVelocityX(speeds.vxMetersPerSecond)
                    .withVelocityY(speeds.vyMetersPerSecond)
                    .withRotationalRate(speeds.omegaRadiansPerSecond)));
  }
  ;

  @Override
  public Command driveRobotCentric(
      DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier rotation) {
    var speeds =
        ChassisSpeeds.discretize(
            calculateDriveVelocity(
                translationX.getAsDouble(), translationY.getAsDouble(), rotation.getAsDouble()),
            DrivetrainConstants.kLoopDt.magnitude());

    return run(
        () ->
            setControl(
                new SwerveRequest.ApplyRobotSpeeds()
                    .withSpeeds(speeds)
                    .withDriveRequestType(DriveRequestType.Velocity)));
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
            () -> {
              var targetSpeeds =
                  new ChassisSpeeds(
                      xPoseController.calculate(getPose().getX(), pose.getX())
                          * DrivetrainConstants.kMaxLinearVelocity,
                      yPoseController.calculate(getPose().getY(), pose.getY())
                          * DrivetrainConstants.kMaxLinearVelocity,
                      thetaController.calculate(
                              getPose().getRotation().getRadians(), pose.getRotation().getRadians())
                          * DrivetrainConstants.kMaxAngularVelocity);

              setControl(new SwerveRequest.ApplyRobotSpeeds().withSpeeds(targetSpeeds));
            });
  }

  // drive with absolute heading control
  // public Command driveFixedHeading(
  //     DoubleSupplier translationX, DoubleSupplier translationY, Supplier<Rotation2d> rotation) {
  //   var speeds =
  //       ChassisSpeeds.discretize(
  //           calculateDriveVelocity(translationX.getAsDouble(), translationY.getAsDouble(), 0),
  //           DrivetrainConstants.kLoopDt.magnitude());

  //   return run(
  //       () ->
  //           setControl(
  //               new SwerveRequest.FieldCentricFacingAngle()
  //                   .withDriveRequestType(DriveRequestType.Velocity)
  //                   .withVelocityX(speeds.vxMetersPerSecond)
  //                   .withVelocityY(speeds.vyMetersPerSecond)
  //                   .withTargetDirection(rotation.get())));
  // }

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

  @Override
  public SwerveModuleState[] getSwerveModuleStates() {
    return super.getState().ModuleStates;
  }

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
