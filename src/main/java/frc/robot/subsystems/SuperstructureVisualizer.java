/* (C) Robolancers 2025 */
package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.elevatorarm.ElevatorArmConstants;
import frc.robot.util.VirtualSubsystem;
import java.util.function.Supplier;

@Logged
public class SuperstructureVisualizer extends VirtualSubsystem {
  Mechanism2d mechanism;
  MechanismLigament2d elevator;
  MechanismLigament2d arm;

  Supplier<Distance> elevatorSetpoint;
  Supplier<Angle> armSetpoint;

  Pose3d elevatorFirstStagePose = new Pose3d();
  Pose3d elevatorSecondStagePose = new Pose3d();
  Pose3d armPose = new Pose3d();
  Pose3d wristPose = new Pose3d();

  public SuperstructureVisualizer(
      Supplier<Distance> elevatorSetpoint, Supplier<Angle> armSetpoint) {
    this.elevatorSetpoint = elevatorSetpoint;
    this.armSetpoint = armSetpoint;

    this.mechanism =
        new Mechanism2d(
            Inches.of(29).in(Meters),
            ElevatorConstants.kElevatorMaximumHeight
                .plus(ElevatorArmConstants.kElevatorArmLength)
                .in(Meters));

    MechanismRoot2d root =
        mechanism.getRoot(
            "Root",
            VisualizerConstants.elevatorRoot2d.getX(),
            VisualizerConstants.elevatorRoot2d.getY());

    this.elevator =
        root.append(
            new MechanismLigament2d(
                "Elevator",
                ElevatorConstants.kElevatorStartingHeight.in(Meters),
                90,
                8.0,
                new Color8Bit(Color.kFirstBlue)));

    this.arm =
        elevator.append(
            new MechanismLigament2d(
                "Arm",
                ElevatorArmConstants.kElevatorArmLength.in(Meters),
                0,
                4.0,
                new Color8Bit(Color.kFirstRed)));
  }

  public void update() {
    // TODO adjust armSetpoint to match our coordinate plane
    arm.setAngle(armSetpoint.get().in(Degrees));
    elevator.setLength(elevatorSetpoint.get().in(Meters));
    SmartDashboard.putData("Mech2d", mechanism);

    // TODO fix elevator pose math
    double firstStageHeight =
        elevatorSetpoint.get().in(Meters) - ElevatorConstants.kElevatorMinimumHeight.in(Meters);
    double secondStageHeight = firstStageHeight / 2;

    this.elevatorFirstStagePose =
        new Pose3d(
            VisualizerConstants.elevatorRoot3d.plus(new Translation3d(0, 0, firstStageHeight)),
            new Rotation3d());

    this.elevatorSecondStagePose =
        new Pose3d(
            VisualizerConstants.elevatorRoot3d.plus(new Translation3d(0, 0, secondStageHeight)),
            new Rotation3d());

    // TODO this is a little greater than elevator height. check math
    this.armPose =
        new Pose3d(
            VisualizerConstants.armRoot3d.plus(
                new Translation3d(
                    0,
                    0,
                    elevator.getLength() - ElevatorConstants.kElevatorMinimumHeight.in(Meters))),
            new Rotation3d());

    // TODO trig functions are switched b/c 0 is straight up. swap when converting to our angles
    this.wristPose =
        armPose.transformBy(
            new Transform3d(
                new Translation3d(
                    0,
                    arm.getLength() * Math.sin(Radians.convertFrom(arm.getAngle(), Degrees)),
                    arm.getLength() * Math.cos(Radians.convertFrom(arm.getAngle(), Degrees))),
                new Rotation3d()));
  }

  @Logged(name = "ElevatorFirstStage")
  public Pose3d getElevatorFirstStagePose() {
    return elevatorFirstStagePose;
  }

  @Logged(name = "ElevatorSecondStage")
  public Pose3d getElevatorSecondStagePose() {
    return elevatorSecondStagePose;
  }

  @Logged(name = "ArmPose")
  public Pose3d getArmPose() {
    return armPose;
  }

  @Logged(name = "WristPose")
  public Pose3d getWristPose() {
    return wristPose;
  }

  @Logged(name = "ElevatorSetpoint")
  public double getElevatorSetpoint() {
    return elevatorSetpoint.get().in(Meters);
  }

  @Logged(name = "ArmSetpoint")
  public double getArmSetpoint() {
    return armSetpoint.get().in(Degrees);
  }

  @Override
  public void periodic() {
    update();
  }
}
