/* (C) Robolancers 2025 */
package frc.robot.subsystems.drivetrain;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import com.pathplanner.lib.util.DriveFeedforwards;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.ReefAlign;
import frc.robot.util.SelfControlledSwerveDriveSimulationWrapper;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.COTS;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;

/*
 * Maplesim drivetrain
 */
@Logged
public class DrivetrainSim implements SwerveDrive {
  private final SelfControlledSwerveDriveSimulationWrapper simulatedDrive;
  private final Field2d field2d;
  private Pose2d alignmentSetpoint = Pose2d.kZero;
  final DriveTrainSimulationConfig simConfig;
  PIDController headingController;

  private final SwerveDrivePoseEstimator reefPoseEstimator;

  public DrivetrainSim() {
    this.simConfig =
        DriveTrainSimulationConfig.Default()
            .withGyro(COTS.ofPigeon2())
            .withSwerveModule(
                COTS.ofMark4(
                    DCMotor.getKrakenX60(1),
                    DCMotor.getKrakenX60(1),
                    COTS.WHEELS.COLSONS.cof, // Use the COF for Colson Wheels
                    3)) // L3 Gear ratio
            // Configures the track length and track width (spacing between swerve modules)
            .withTrackLengthTrackWidth(
                DrivetrainConstants.kWheelBase, DrivetrainConstants.kTrackWidth)
            // Configures the bumper size (dimensions of the robot bumper) trackwidth + 6 inches
            .withBumperSize(
                DrivetrainConstants.kWheelBase.plus(Inches.of(6)),
                DrivetrainConstants.kTrackWidth.plus(Inches.of(6)))
            .withRobotMass(Pounds.of(113));

    this.simulatedDrive =
        new SelfControlledSwerveDriveSimulationWrapper(
            new SwerveDriveSimulation(simConfig, new Pose2d(2, 2, new Rotation2d())));

    this.headingController =
        new PIDController(
            DrivetrainConstants.tuneHeadingGains.kP(),
            DrivetrainConstants.tuneHeadingGains.kI(),
            DrivetrainConstants.tuneHeadingGains.kD());

    this.headingController.enableContinuousInput(-Math.PI, Math.PI);

    SimulatedArena.getInstance().addDriveTrainSimulation(simulatedDrive.getDriveTrainSimulation());

    // A field2d widget for debugging
    field2d = new Field2d();
    SmartDashboard.putData("simulation field", field2d);

    this.reefPoseEstimator =
        new SwerveDrivePoseEstimator(
            simulatedDrive.getKinematics(), getHeading(), getModulePositions(), getPose());

    configureAutoBuilder();
    configurePoseControllers();
  }

  @Override
  public Command teleopDrive(
      DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier rotation) {
    return run(
        () -> {
          ChassisSpeeds speeds =
              flipFieldSpeeds(
                  new ChassisSpeeds(
                      translationX.getAsDouble(),
                      translationY.getAsDouble(),
                      rotation.getAsDouble()));
          simulatedDrive.runChassisSpeeds(speeds, new Translation2d(), true, false);
        });
  }

  @Override
  public Command teleopDriveFixedHeading(
      DoubleSupplier translationX,
      DoubleSupplier translationY,
      DoubleSupplier rotationX,
      DoubleSupplier rotationY) {
    return run(
        () -> {
          Rotation2d desiredRotation =
              flipRotation(
                  Rotation2d.fromRadians(
                      Math.atan2(rotationX.getAsDouble(), rotationY.getAsDouble())));

          ChassisSpeeds speeds =
              flipFieldSpeeds(
                  new ChassisSpeeds(
                      translationX.getAsDouble(),
                      translationY.getAsDouble(),
                      headingController.calculate(
                          getPose().getRotation().getRadians(), desiredRotation.getRadians())));

          simulatedDrive.runChassisSpeeds(speeds, new Translation2d(), true, false);
        });
  }

  @Override
  public Command driveFieldCentric(
      DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier rotation) {

    return run(
        () ->
            simulatedDrive.runChassisSpeeds(
                new ChassisSpeeds(
                    translationX.getAsDouble(), translationY.getAsDouble(), rotation.getAsDouble()),
                new Translation2d(),
                true,
                false));
  }

  @Override
  public Command driveRobotCentric(
      DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier rotation) {
    return run(
        () ->
            driveRobotCentric(
                translationX.getAsDouble(),
                translationY.getAsDouble(),
                rotation.getAsDouble(),
                DriveFeedforwards.zeros(4)));
  }

  @Override
  public void driveRobotCentric(
      double translationX, double translationY, double rotation, DriveFeedforwards feedforwards) {
    simulatedDrive.runChassisSpeeds(
        new ChassisSpeeds(translationX, translationY, rotation), new Translation2d(), false, false);
  }

  @Override
  public Command driveToRobotPose(Supplier<Pose2d> pose) {
    return runOnce(
            () -> {
              xPoseController.reset();
              yPoseController.reset();
              thetaController.reset();
            })
        .andThen(
            run(
                () -> {
                  ChassisSpeeds targetSpeeds =
                      new ChassisSpeeds(
                          xPoseController.calculate(getPose().getX(), pose.get().getX())
                              * DrivetrainConstants.kMaxLinearVelocity.in(MetersPerSecond),
                          yPoseController.calculate(getPose().getY(), pose.get().getY())
                              * DrivetrainConstants.kMaxLinearVelocity.in(MetersPerSecond),
                          thetaController.calculate(
                                  getPose().getRotation().getRadians(),
                                  pose.get().getRotation().getRadians())
                              * DrivetrainConstants.kMaxAngularVelocity.in(RadiansPerSecond));

                  driveRobotCentric(
                      targetSpeeds.vxMetersPerSecond,
                      targetSpeeds.vyMetersPerSecond,
                      targetSpeeds.omegaRadiansPerSecond,
                      DriveFeedforwards.zeros(4));
                }));
  }

  @Override
  public void driveToFieldPose(Pose2d pose) {

    if (pose == null) return;

    ChassisSpeeds targetSpeeds =
        new ChassisSpeeds(
            xPoseController.calculate(getPose().getX(), pose.getX())
                * DrivetrainConstants.kMaxLinearVelocity.in(MetersPerSecond),
            yPoseController.calculate(getPose().getY(), pose.getY())
                * DrivetrainConstants.kMaxLinearVelocity.in(MetersPerSecond),
            thetaController.calculate(
                    getPose().getRotation().getRadians(), pose.getRotation().getRadians())
                * DrivetrainConstants.kMaxAngularVelocity.in(RadiansPerSecond));

    if (atPoseSetpoint()) targetSpeeds = new ChassisSpeeds();

    simulatedDrive.runChassisSpeeds(targetSpeeds, Translation2d.kZero, true, false);
  }

  @Override
  public void driveFixedHeading(double translationX, double translationY, Rotation2d rotation) {
    ChassisSpeeds speeds =
        flipFieldSpeeds(
            new ChassisSpeeds(
                translationX,
                translationY,
                headingController.calculate(
                    getPose().getRotation().getRadians(), rotation.getRadians())));

    simulatedDrive.runChassisSpeeds(speeds, new Translation2d(), true, false);
  }

  @Override
  public void resetPose(Pose2d pose) {
    simulatedDrive.resetOdometry(pose);
  }

  @Override
  public void setAlignmentSetpoint(Pose2d setpoint) {
    alignmentSetpoint = setpoint;
  }

  @Override
  public boolean atPoseSetpoint() {
    final var currentPose = getPose();
    return currentPose.getTranslation().getDistance(alignmentSetpoint.getTranslation())
            < DrivetrainConstants.kAlignmentSetpointTranslationTolerance.in(Meters)
        && Math.abs(currentPose.getRotation().minus(alignmentSetpoint.getRotation()).getDegrees())
            < DrivetrainConstants.kAlignmentSetpointRotationTolerance.in(Degrees);
  }

  @Override
  public void setSwerveModuleStates(SwerveModuleState[] states) {
    simulatedDrive.runSwerveStates(states);
  }

  @Logged(name = "MeasuredModuleStates")
  @Override
  public SwerveModuleState[] getMeasuredModuleStates() {
    return simulatedDrive.getMeasuredStates();
  }

  @Logged(name = "MeasuredModulePositions")
  @Override
  public SwerveModulePosition[] getModulePositions() {
    return simulatedDrive.getLatestModulePositions();
  }

  @Logged(name = "TargetModuleStates")
  @Override
  public SwerveModuleState[] getTargetModuleStates() {
    return simulatedDrive.getSetPointsOptimized();
  }

  @Logged(name = "ReefVisionEstimatedPose")
  @Override
  public Pose2d getReefVisionPose() {
    return reefPoseEstimator.getEstimatedPosition();
  }

  @Logged(name = "MeasuredRobotPose")
  @Override
  public Pose2d getPose() {
    return simulatedDrive.getOdometryEstimatedPose();
  }

  @Logged(name = "ActualRobotPose")
  public Pose2d getActualPose() {
    return simulatedDrive.getActualPoseInSimulationWorld();
  }

  @Logged(name = "MeasuredRobotRelativeChassisSpeeds")
  @Override
  public ChassisSpeeds getChassisSpeeds() {
    return simulatedDrive.getMeasuredSpeedsRobotRelative(false);
  }

  @Logged(name = "MeasuredHeading")
  @Override
  public Rotation2d getHeading() {
    return simulatedDrive.getDriveTrainSimulation().getGyroSimulation().getGyroReading();
  }

  @Override
  public void addVisionMeasurement(Pose2d visionRobotPose, double timeStampSeconds) {
    simulatedDrive.addVisionEstimation(visionRobotPose, timeStampSeconds);
  }

  @Override
  public void addVisionMeasurement(
      Pose2d visionRobotPose, double timeStampSeconds, Matrix<N3, N1> standardDeviations) {
    simulatedDrive.addVisionEstimation(visionRobotPose, timeStampSeconds, standardDeviations);
  }

  @Override
  public void addReefVisionMeasurement(
      Pose2d visionRobotPose, double timeStampSeconds, Matrix<N3, N1> standardDeviations) {
    reefPoseEstimator.addVisionMeasurement(visionRobotPose, timeStampSeconds, standardDeviations);
  }

  @Override
  public void periodic() {
    // update simulated drive and arena
    SimulatedArena.getInstance().simulationPeriodic();
    simulatedDrive.periodic();

    reefPoseEstimator.update(getHeading(), getModulePositions());

    // send simulation data to dashboard for testing
    field2d.setRobotPose(simulatedDrive.getActualPoseInSimulationWorld());
    field2d.getObject("odometry").setPose(getPose());
  }

  @Logged(name = "RobotLeftAligned")
  public Pose2d robotLeftAligned() {
    return ReefAlign.leftAlignPoses.get(ReefAlign.getNearestReefID(getPose()));
  }

  @Logged(name = "RobotRightAligned")
  public Pose2d robotRightAligned() {
    return ReefAlign.rightAlignPoses.get(ReefAlign.getNearestReefID(getPose()));
  }

  @Logged(name = "ClosestAprilTag")
  public Pose2d closestAprilTag() {
    return ReefAlign.getNearestReefPose(getPose());
  }
}
