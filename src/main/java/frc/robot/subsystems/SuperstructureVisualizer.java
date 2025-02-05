/* (C) Robolancers 2025 */
package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.elevatorarm.ElevatorArmConstants;
import frc.robot.util.VirtualSubsystem;
import java.util.function.DoubleSupplier;

public class SuperstructureVisualizer extends VirtualSubsystem {
  Mechanism2d mechanism;
  MechanismLigament2d elevator;
  MechanismLigament2d arm;

  Translation2d superstructureRoot2d =
      new Translation2d(Inches.of(1).in(Meters), Inches.of(1).in(Meters));
  Translation3d superstructureRoot3d = new Translation3d(superstructureRoot2d);

  DoubleSupplier elevatorSetpoint = () -> ElevatorConstants.kElevatorStartingHeight.in(Meters);
  DoubleSupplier armSetpoint = () -> Degrees.of(0).in(Degrees);
  Pose3d elevatorPose = new Pose3d();
  Pose3d armPose = new Pose3d();

  StructArrayPublisher<Pose3d> arrayPublisher =
      NetworkTableInstance.getDefault().getStructArrayTopic("MyPoseArray", Pose3d.struct).publish();

  public SuperstructureVisualizer(DoubleSupplier elevatorSetpoint, DoubleSupplier armSetpoint) {
    this.elevatorSetpoint = elevatorSetpoint;
    this.armSetpoint = armSetpoint;

    this.mechanism =
        new Mechanism2d(
            Inches.of(29).in(Meters), ElevatorConstants.kElevatorMaximumHeight.in(Meters));
    MechanismRoot2d root =
        mechanism.getRoot(
            "Elevator Root", superstructureRoot2d.getX(), superstructureRoot2d.getY());

    elevator =
        root.append(
            new MechanismLigament2d(
                "Elevator",
                ElevatorConstants.kElevatorStartingHeight.in(Meters),
                90,
                8.0,
                new Color8Bit(Color.kFirstBlue)));

    arm =
        elevator.append(
            new MechanismLigament2d(
                "ElevatorArm",
                ElevatorArmConstants.kElevatorArmLength.in(Meters),
                180,
                4.0,
                new Color8Bit(Color.kFirstRed)));

    SmartDashboard.putData("Mech2d", mechanism);
  }

  public void update() {
    elevator.setLength(
        ElevatorConstants.kElevatorMinimumHeight.in(Meters) + elevatorSetpoint.getAsDouble());
    arm.setAngle(armSetpoint.getAsDouble());

    elevatorPose =
        new Pose3d(
            superstructureRoot3d.plus(
                new Translation3d(elevatorSetpoint.getAsDouble(), new Rotation3d(0, 90, 0))),
            new Rotation3d());

    armPose =
        elevatorPose.transformBy(
            new Transform3d(
                new Translation3d(0, arm.getLength(), elevator.getLength()),
                new Rotation3d(arm.getAngle(), 0, 0)));

    arrayPublisher.set(new Pose3d[] {elevatorPose, armPose});
  }

  @Override
  public void periodic() {
    update();
  }
}
