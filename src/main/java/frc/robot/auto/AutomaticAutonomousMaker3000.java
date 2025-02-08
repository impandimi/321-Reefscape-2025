/* (C) Robolancers 2025 */
package frc.robot.auto;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.CoralSuperstructure;
import frc.robot.subsystems.coralendeffector.CoralEndEffector;
import java.util.ArrayList;

public class AutomaticAutonomousMaker3000 {

  private SendableChooser<StartingPosition> startingPosition = new SendableChooser<>();

  private ArrayList<ScoringGroup> scoringGroups = new ArrayList<>();

  public AutomaticAutonomousMaker3000(
      CoralEndEffector coralEndEffector, CoralSuperstructure coralSuperstructure) {

    startingPosition.setDefaultOption("Top", StartingPosition.TOP);
    startingPosition.addOption("Middle", StartingPosition.MIDDLE);
    startingPosition.addOption("Bottom", StartingPosition.BOTTOM);

    SmartDashboard.putData("Starting Position", startingPosition);

    for (int i = 0; i < 6; i++) {
      scoringGroups.add(new ScoringGroup(i));
    }
  }

  public Command buildAuto(
      CoralEndEffector coralEndEffector, CoralSuperstructure coralSuperstructure) {

    Command auto = Commands.none();

    ReefSide lastReefSide = scoringGroups.get(0).reefSide.getSelected();

    for (int i = 0; i < scoringGroups.size(); i++) {

      if ((i != 0 && scoringGroups.get(i).feedLocation.getSelected() == FeedLocation.NOCHOICE)
          || scoringGroups.get(i).reefSide.getSelected() == ReefSide.NOCHOICE) break;
      if (i == 0) {
        // first value; score preload and ignore the alt destination instructions
        auto =
            auto.andThen(
                withScoring(
                    getPathCommand(
                        startingPosition.getSelected().pathID
                            + " to "
                            + scoringGroups.get(i).reefSide.getSelected().pathID),
                    coralSuperstructure));
      } else {

        auto =
            auto.andThen(
                    withIntaking(
                        getPathCommand(
                            lastReefSide.pathID
                                + " to "
                                + scoringGroups.get(i).feedLocation.getSelected().pathID),
                        coralEndEffector))
                .andThen(
                    withScoring(
                        getPathCommand(
                            scoringGroups.get(i).feedLocation.getSelected().pathID
                                + " to "
                                + scoringGroups.get(i).reefSide.getSelected().pathID),
                        coralSuperstructure));
        lastReefSide = scoringGroups.get(i).reefSide.getSelected();
      }
    }

    return auto;
  }

  public Command withIntaking(Command path, CoralEndEffector coralendeffector) {
    return path;
    // return path.alongWith(coralendeffector.intakeCoral()).until(coralendeffector::hasCoral);
  }

  public Command withScoring(Command path, CoralSuperstructure coralSuperstructure) {
    return path;
    // return path.andThen(coralSuperstructure.outtakeCoral());
  }

  public Command getPathCommand(String pathName) {
    System.out.println(pathName);
    try {
      // Load the path you want to follow using its name in the GUI
      PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);

      // Create a path following command using AutoBuilder. This will also trigger event markers.
      return AutoBuilder.followPath(path);
    } catch (Exception e) {
      DriverStation.reportError("Big oops: " + e.getMessage(), e.getStackTrace());
      return Commands.none();
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
    // LEFTCORAL1("LeftCoral1"),
    // LEFTCORAL2("LeftCoral2"),
    // LEFTCORAL3("LeftCoral3"),
    UPCORAL("UpCoral"),
    DOWNCORAL("DownCoral");

    private String pathID;

    FeedLocation(String pathID) {
      this.pathID = pathID;
    }
  }

  public class ScoringGroup {
    // add sendable choosers
    private SendableChooser<ReefSide> reefSide = new SendableChooser<>();
    private SendableChooser<Level> level = new SendableChooser<>();
    private SendableChooser<Pole> pole = new SendableChooser<>();
    private SendableChooser<FeedLocation> feedLocation = new SendableChooser<>();

    public ScoringGroup(int index) {

      reefSide.setDefaultOption("None", ReefSide.NOCHOICE);
      reefSide.addOption("Far Left", ReefSide.REEFR1);
      reefSide.addOption("Far", ReefSide.REEFR2);
      reefSide.addOption("Far Right", ReefSide.REEFR3);
      reefSide.addOption("Close Left", ReefSide.REEFL1);
      reefSide.addOption("Close", ReefSide.REEFL2);
      reefSide.addOption("Close Right", ReefSide.REEFL3);

      level.setDefaultOption("No Choice", Level.NOCHOICE);
      level.addOption("L1", Level.L1);
      level.addOption("L2", Level.L2);
      level.addOption("L3", Level.L3);
      level.addOption("L4", Level.L4);

      pole.setDefaultOption("Right", Pole.RIGHTPOLE);
      pole.addOption("Left", Pole.LEFTPOLE);

      feedLocation.setDefaultOption("No Choice", FeedLocation.NOCHOICE);
      // feedLocation.addOption("Left Coral 1", FeedLocation.LEFTCORAL1);
      // feedLocation.addOption("Left Coral 2", FeedLocation.LEFTCORAL2);
      // feedLocation.addOption("Left Coral 3", FeedLocation.LEFTCORAL3);
      feedLocation.addOption("Right Station", FeedLocation.DOWNCORAL);
      feedLocation.addOption("Left Station", FeedLocation.UPCORAL);

      SmartDashboard.putData("Reef Side" + index, reefSide);
      SmartDashboard.putData("Level" + index, level);
      SmartDashboard.putData("Pole" + index, pole);
      SmartDashboard.putData("Alternate" + index, feedLocation);
    }
  }
}
