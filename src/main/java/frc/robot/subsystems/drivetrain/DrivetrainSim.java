/* (C) Robolancers 2025 */
package frc.robot.subsystems.drivetrain;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Pounds;

import com.pathplanner.lib.util.DriveFeedforwards;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.DoubleSupplier;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.COTS;
import org.ironmaple.simulation.drivesims.SelfControlledSwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;

/*
 * Maplesim drivetrain
 */
@Logged
public class DrivetrainSim implements SwerveDrive {
  // sim
  private final SelfControlledSwerveDriveSimulation simulatedDrive;
  private final Field2d field2d;
  final DriveTrainSimulationConfig simConfig;

  // drive to pose controllers

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
            .withTrackLengthTrackWidth(Inches.of(29), Inches.of(29))
            // Configures the bumper size (dimensions of the robot bumper) trackwidth + 6 inches
            .withBumperSize(Inches.of(35), Inches.of(35))
            .withRobotMass(Pounds.of(113));

    this.simulatedDrive =
        new SelfControlledSwerveDriveSimulation(
            new SwerveDriveSimulation(simConfig, new Pose2d(2, 2, new Rotation2d())));

    SimulatedArena.getInstance().addDriveTrainSimulation(simulatedDrive.getDriveTrainSimulation());

    // A field2d widget for debugging
    field2d = new Field2d();
    SmartDashboard.putData("simulation field", field2d);

    configureAutoBuilder();
    configureControllers();

    // this.moduleStatePublisher =
    //   NetworkTableInstance.getDefault()
    //       .getStructArrayTopic("/SwerveCurrentState", SwerveModuleState.struct)
    //       .publish();
  }

  @Override
  public Command driveAuto(ChassisSpeeds speeds, DriveFeedforwards feedforward) {
    return run(() -> simulatedDrive.runChassisSpeeds(speeds, new Translation2d(), false, false));
  }

  @Override
  public Command driveFieldCentric(
      DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier rotation) {
    var speeds =
        calculateDriveVelocity(
            translationX.getAsDouble(), translationY.getAsDouble(), rotation.getAsDouble());
    return run(() -> simulatedDrive.runChassisSpeeds(speeds, new Translation2d(), true, true));
  }

  @Override
  public Command driveRobotCentric(
      DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier rotation) {
    var speeds =
        calculateDriveVelocity(
            translationX.getAsDouble(), translationY.getAsDouble(), rotation.getAsDouble());

    return run(() -> simulatedDrive.runChassisSpeeds(speeds, new Translation2d(), false, true));
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
              ChassisSpeeds targetSpeeds =
                  new ChassisSpeeds(
                      xPoseController.calculate(getPose().getX(), pose.getX())
                          * DrivetrainConstants.kMaxLinearVelocity,
                      yPoseController.calculate(getPose().getY(), pose.getY())
                          * DrivetrainConstants.kMaxLinearVelocity,
                      thetaController.calculate(
                              getPose().getRotation().getRadians(), pose.getRotation().getRadians())
                          * DrivetrainConstants.kMaxAngularVelocity);

              driveRobotCentric(
                  () -> targetSpeeds.vxMetersPerSecond,
                  () -> targetSpeeds.vyMetersPerSecond,
                  () -> targetSpeeds.omegaRadiansPerSecond);
            });
  }

  @Override
  public void resetPose(Pose2d pose) {
    simulatedDrive.resetOdometry(pose);
  }

  @Override
  public void setSwerveModuleStates(SwerveModuleState[] states) {
    simulatedDrive.runSwerveStates(states);
  }

  @Override
  public SwerveModuleState[] getSwerveModuleStates() {
    return simulatedDrive.getMeasuredStates();
  }

  @Override
  public SwerveModuleState[] getTargetModuleStates() {
    return simulatedDrive.getSetPointsOptimized();
  }

  @Override
  public Pose2d getPose() {
    return simulatedDrive.getOdometryEstimatedPose();
  }

  @Override
  public ChassisSpeeds getChassisSpeeds() {
    return simulatedDrive.getActualSpeedsRobotRelative();
  }

  @Override
  public Rotation2d getHeading() {
    return simulatedDrive.getDriveTrainSimulation().getGyroSimulation().getGyroReading();
  }

  @Override
  public void addVisionMeasurement(Pose2d visionRobotPose, double timeStampSeconds) {
    simulatedDrive.addVisionEstimation(visionRobotPose, timeStampSeconds);
  }

  @Override
  public void periodic() {
    // update simulated drive and arena
    SimulatedArena.getInstance().simulationPeriodic();
    simulatedDrive.periodic();

    // send simulation data to dashboard for testing
    field2d.setRobotPose(simulatedDrive.getActualPoseInSimulationWorld());
    field2d.getObject("odometry").setPose(getPose());
  }
}
