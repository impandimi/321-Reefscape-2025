/* (C) Robolancers 2025 */
package frc.robot.subsystems.drivetrain;

import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SelfControlledSwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;

public class DrivetrainSim extends Drivetrain {
  private final SelfControlledSwerveDriveSimulation simulatedDrive;
  private final Field2d field2d;

  public DrivetrainSim(
      SwerveDrivetrainConstants constants, SwerveModuleConstants<?, ?, ?>... modules) {
    super(constants, modules);
    final DriveTrainSimulationConfig config = DriveTrainSimulationConfig.Default();

    this.simulatedDrive =
        new SelfControlledSwerveDriveSimulation(
            new SwerveDriveSimulation(config, new Pose2d(0, 0, new Rotation2d())));

    SimulatedArena.getInstance().addDriveTrainSimulation(simulatedDrive.getDriveTrainSimulation());

    // A field2d widget for debugging
    field2d = new Field2d();
    SmartDashboard.putData("simulation field", field2d);
  }

  @Override
  public Command driveFieldCentric(double translationX, double translationY, double rotation) {
    return run(
        () -> {
          this.simulatedDrive.runChassisSpeeds(
              new ChassisSpeeds(translationX, translationY, rotation),
              new Translation2d(),
              true,
              true);
        });
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
  public void simulationPeriodic() {
    simulatedDrive.periodic();

    // send simulation data to dashboard for testing
    field2d.setRobotPose(simulatedDrive.getActualPoseInSimulationWorld());
    field2d.getObject("odometry").setPose(getPose());

    // CTR sim
    this.updateSimState(0.02, RobotController.getBatteryVoltage());
  }
}
