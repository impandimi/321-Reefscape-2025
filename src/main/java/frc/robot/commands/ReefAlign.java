/* (C) Robolancers 2025 */
package frc.robot.commands;

import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.drivetrain.SwerveDrive;
import java.util.List;
import java.util.Optional;
import java.util.function.DoubleSupplier;

public class ReefAlign {
  private static final Distance kLeftAlignDistance = Inches.of(-6.5);
  private static final Distance kReefDistance = Inches.of(14.5);
  private static final Distance kRightAlignDistance = Inches.of(6.5);
  private static final Rotation2d kReefAlignmentRotation = Rotation2d.fromDegrees(180);

  private static final List<Integer> blueReefIDs = List.of(17, 18, 19, 20, 21, 22);
  private static final List<Integer> redReefIDs = List.of(6, 7, 8, 9, 10, 11);
  private static final List<AprilTag> blueReefTags =
      Constants.kAprilTagFieldLayout.getTags().stream()
          .filter(tag -> blueReefIDs.contains(tag.ID))
          .toList();
  private static final List<AprilTag> redReefTags =
      Constants.kAprilTagFieldLayout.getTags().stream()
          .filter(tag -> redReefIDs.contains(tag.ID))
          .toList();

  public static Pose2d getClosestReefPose(Pose2d robotPose) {
    Optional<Alliance> alliance = DriverStation.getAlliance();

    if (alliance.isEmpty()) return null;

    if (alliance.get().equals(Alliance.Blue)) {
      return robotPose.nearest(blueReefTags.stream().map(tag -> tag.pose.toPose2d()).toList());
    } else if (alliance.get().equals(Alliance.Red)) {
      return robotPose.nearest(redReefTags.stream().map(tag -> tag.pose.toPose2d()).toList());
    }

    return null;
  }

  public static int getClosestReefID(Pose2d robotPose) {
    Pose2d closest = getClosestReefPose(robotPose);

    if (closest == null) return -1;

    return Constants.kAprilTagFieldLayout.getTags().stream()
        .filter(tag -> tag.pose.toPose2d().equals(closest))
        .toList()
        .get(0)
        .ID;
  }

  public static Pose2d getNearestLeftAlign(int reefTagID) {
    Optional<Pose3d> tagPose = Constants.kAprilTagFieldLayout.getTagPose(reefTagID);

    if (tagPose.isEmpty()) return null;

    Pose2d aprilTagPose = tagPose.get().toPose2d();

    Pose2d resultPose =
        aprilTagPose.plus(
            new Transform2d(kReefDistance, kLeftAlignDistance, kReefAlignmentRotation));

    // new Pose2d(kReefDistance, kLeftAlignDistance, kReefAlignmentRotation)
    //         .relativeTo(aprilTagPose);
    return resultPose;
  }

  public static Pose2d getNearestRightAlign(int reefTagID) {
    Optional<Pose3d> tagPose = Constants.kAprilTagFieldLayout.getTagPose(reefTagID);

    if (tagPose.isEmpty()) return null;

    Pose2d aprilTagPose = tagPose.get().toPose2d();

    Pose2d resultPose =
        aprilTagPose.plus(
            new Transform2d(kReefDistance, kRightAlignDistance, kReefAlignmentRotation));

    // new Pose2d(kReefDistance, kRightAlignDistance, kReefAlignmentRotation)
    //         .relativeTo(aprilTagPose);
    return resultPose;
  }

  public static Command goToNearestLeftAlign(SwerveDrive swerveDrive) {
    return swerveDrive.driveToFieldPose(
        () -> getNearestLeftAlign(getClosestReefID(getClosestReefPose(swerveDrive.getPose()))));
  }

  public static Command goToNearestRightAlign(SwerveDrive swerveDrive) {
    return swerveDrive.driveToFieldPose(
        () -> getNearestRightAlign(getClosestReefID(getClosestReefPose(swerveDrive.getPose()))));
  }

  public static Command rotateToNearest(
      SwerveDrive swerveDrive, DoubleSupplier x, DoubleSupplier y) {
    return swerveDrive.driveFixedHeading(
        x,
        y,
        () -> (getClosestReefPose(swerveDrive.getPose()).getRotation().plus(Rotation2d.k180deg)));
  }
}
