/* (C) Robolancers 2025 */
package frc.robot.auto;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.CoralSuperstructure;
import java.util.ArrayList;
import java.util.List;

public class AutomaticAutonomousMaker3000 {

  private CycleAutoChooser autoChooser = new CycleAutoChooser(3);

  Field2d field = new Field2d();

  private CoralSuperstructure coralSuperstructure;

  private Command storedAuto; 

  public AutomaticAutonomousMaker3000(CoralSuperstructure coralSuperstructure) {
    this.coralSuperstructure = coralSuperstructure;

    SmartDashboard.putData("Submit", Commands.runOnce(() -> {
        storedAuto = buildAuto(autoChooser.build()).getAuto();
    }));
  }

  public Command getStoredAuto() {
    return storedAuto;
  }
  private void visualizeAuto(List<PathPlannerPath> paths) {
    List<Pose2d> visualizePath = new ArrayList<>();

    for (int i = 0; i < paths.size(); i++) {
        visualizePath.addAll(paths.get(i).getPathPoses()); 
    }

    SmartDashboard.putData("Field", field);

    field.getObject("PathPoses").setPoses(visualizePath);
  }

  public PathAndAuto buildAuto(CycleAutoConfig config) {

    Command auto = Commands.none();
    List<PathPlannerPath> paths = new ArrayList<>();

    ReefSide lastReefSide = config.scoringGroup.get(0).reefSide;

    for (int i = 0; i < config.scoringGroup.size(); i++) {

      if ((i != 0 && config.scoringGroup.get(i).feedLocation == FeedLocation.NOCHOICE)
          || config.scoringGroup.get(i).reefSide == ReefSide.NOCHOICE) break;
      if (i == 0) {
        // first value; score preload and ignore the alt destination instructions
        PathPlannerPath path =
            getPath(
                config.startingPosition.pathID
                    + " to "
                    + config.scoringGroup.get(i).reefSide.pathID);
        auto = auto.andThen(withScoring(toPathCommand(path)));
        paths.add(path);
      } else {

        PathPlannerPath intakePath =
            getPath(lastReefSide.pathID + " to " + config.scoringGroup.get(i).feedLocation.pathID);

        PathPlannerPath scorePath =
            getPath(
                config.scoringGroup.get(i).feedLocation.pathID
                    + " to "
                    + config.scoringGroup.get(i).reefSide.pathID);

        auto =
            auto.andThen(withIntaking(toPathCommand(intakePath)))
                .andThen(withScoring(toPathCommand(scorePath)));
        lastReefSide = config.scoringGroup.get(i).reefSide;

        paths.add(intakePath);
        paths.add(scorePath);
      }
    }
    return new PathAndAuto(auto, paths);
  }

  public Command withIntaking(Command path) {
    // TODO: add intaking logic
    return path;
    // return path.alongWith(coralendeffector.intakeCoral()).until(coralendeffector::hasCoral);
  }

  public Command withScoring(Command path) {
    // TODO: add scoring logic
    return path;
    // return path.andThen(coralSuperstructure.outtakeCoral());
  }

  private Command toPathCommand(PathPlannerPath path) {
    if (path == null) return Commands.none();
    return AutoBuilder.followPath(path);
  }

  private PathPlannerPath getPath(String pathName) {
    try {
      // Load the path you want to follow using its name in the GUI
      return PathPlannerPath.fromPathFile(pathName);
    } catch (Exception e) {
      DriverStation.reportError("Big oops: " + e.getMessage(), e.getStackTrace());
      return null;
    }
  }

  enum StartingPosition {
    TOP("Starting 1"),
    MIDDLE("Starting 2"),
    BOTTOM("Starting 3");

    private String pathID;

    StartingPosition(String pathID) {
      this.pathID = pathID;
    }
  }

  enum ReefSide {
    NOCHOICE("Brake"),
    REEFR1("ReefR1"),
    REEFR2("ReefR2"),
    REEFR3("ReefR3"),
    REEFL1("ReefL1"),
    REEFL2("ReefL2"),
    REEFL3("ReefL3");

    private String pathID;

    ReefSide(String pathID) {
      this.pathID = pathID;
    }
  }

  enum Level {
    NOCHOICE,
    L1,
    L2,
    L3,
    L4;
  }

  enum Pole {
    NOCHOICE,
    LEFTPOLE,
    RIGHTPOLE;
  }

  enum FeedLocation {
    NOCHOICE("Brake"),
    UPCORAL("UpCoral"),
    DOWNCORAL("DownCoral");

    private String pathID = "";

    FeedLocation(String pathID) {
      this.pathID = pathID;
    }
  }

  public static class ScoringGroupChooser {
    // add sendable choosers
    private SendableChooser<ReefSide> reefSide = new SendableChooser<>();
    private SendableChooser<Level> level = new SendableChooser<>();
    private SendableChooser<Pole> pole = new SendableChooser<>();
    private SendableChooser<FeedLocation> feedLocation = new SendableChooser<>();

    public ScoringGroupChooser(int index) {

      reefSide.setDefaultOption("No Choice", ReefSide.NOCHOICE);
      reefSide.addOption("ReefR1", ReefSide.REEFR1);
      reefSide.addOption("ReefR2", ReefSide.REEFR2);
      reefSide.addOption("ReefR3", ReefSide.REEFR3);
      reefSide.addOption("ReefL1", ReefSide.REEFL1);
      reefSide.addOption("ReefL2", ReefSide.REEFL2);
      reefSide.addOption("ReefL3", ReefSide.REEFL3);

      level.setDefaultOption("No Choice", Level.NOCHOICE);
      level.addOption("L1", Level.L1);
      level.addOption("L2", Level.L2);
      level.addOption("L3", Level.L3);
      level.addOption("L4", Level.L4);

      pole.setDefaultOption("Right", Pole.RIGHTPOLE);
      pole.addOption("Left", Pole.LEFTPOLE);

      feedLocation.setDefaultOption("No Choice", FeedLocation.NOCHOICE);
      feedLocation.addOption("Down Coral", FeedLocation.DOWNCORAL);
      feedLocation.addOption("Up Coral", FeedLocation.UPCORAL);

      SmartDashboard.putData("Reef Side" + index, reefSide);
      SmartDashboard.putData("Level" + index, level);
      SmartDashboard.putData("Pole" + index, pole);
      SmartDashboard.putData("FeedLocation" + index, feedLocation);
    }

    public ScoringGroup build() {
      return new ScoringGroup(
          feedLocation.getSelected(),
          pole.getSelected(),
          reefSide.getSelected(),
          level.getSelected());
    }
  }

  public static class ScoringGroup {
    private FeedLocation feedLocation;
    private Pole pole;
    private ReefSide reefSide;
    private Level level;
    private StartingPosition startingPosition;

    public ScoringGroup(FeedLocation feedLocation, Pole pole, ReefSide reefSide, Level level) {
      this.feedLocation = feedLocation;
      this.pole = pole;
      this.reefSide = reefSide;
      this.level = level;
    }
  }

  public static class CycleAutoConfig {
    private List<ScoringGroup> scoringGroup = new ArrayList<>();
    private StartingPosition startingPosition;

    public CycleAutoConfig(StartingPosition startingPosition, List<ScoringGroup> scoringGroup) {
      this.scoringGroup = scoringGroup;
      this.startingPosition = startingPosition;
    }
  }

  public static class CycleAutoChooser {
    private SendableChooser<StartingPosition> startingPosition = new SendableChooser<>();
    private List<ScoringGroupChooser> sgChoosers = new ArrayList<>();

    public CycleAutoChooser(int chooserSize) {

      startingPosition.setDefaultOption("Top", StartingPosition.TOP);
      startingPosition.addOption("Middle", StartingPosition.MIDDLE);
      startingPosition.addOption("Bottom", StartingPosition.BOTTOM);

      SmartDashboard.putData("Starting Position", startingPosition);

      for (int i = 0; i < chooserSize; i++) sgChoosers.add(new ScoringGroupChooser(i));
    }

    public CycleAutoConfig build() {
      return new CycleAutoConfig(
          startingPosition.getSelected(), sgChoosers.stream().map(a -> a.build()).toList());
    }
  }

  public class PathAndAuto {
    Command auto;
    List<PathPlannerPath> path;

    public PathAndAuto(Command auto, List<PathPlannerPath> path) {
        this.auto = auto;
        this.path = path;
    }

    public Command getAuto() {
        return auto;
    }

    public List<PathPlannerPath> getPath() {
        return path;
    }
  }
}