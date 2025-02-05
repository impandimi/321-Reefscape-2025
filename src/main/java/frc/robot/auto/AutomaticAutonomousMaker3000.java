package frc.robot.auto;

import java.util.ArrayList;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.auto.AutomaticAutonomousMaker3000.ScoringGroup;

public class AutomaticAutonomousMaker3000 {
    
    private SendableChooser<StartingPosition> startingPosition;
    private SendableChooser<ReefSide> reefSide;
    

    private ArrayList<ScoringGroup> scoringGroups = new ArrayList<>(); 

    public AutomaticAutonomousMaker3000() {

        startingPosition.setDefaultOption("Top", StartingPosition.TOP);
        startingPosition.addOption("Middle", StartingPosition.MIDDLE);
        startingPosition.addOption("Bottom", StartingPosition.BOTTOM);

        reefSide.setDefaultOption("Null", ReefSide.NOCHOICE);
        reefSide.addOption("ReefL1", ReefSide.REEFL1);
        reefSide.addOption("ReefL2", ReefSide.REEFL2);
        reefSide.addOption("ReefL3", ReefSide.REEFL3); 
        reefSide.addOption("ReefR1", ReefSide.REEFR1);
        reefSide.addOption("ReefR2", ReefSide.REEFR2);
        reefSide.addOption("ReefR3", ReefSide.REEFR3);
        
        SmartDashboard.putData("Starting Position", startingPosition);
 
        for (int i = 0; i < 6; i++) {
            scoringGroups.add(new ScoringGroup()); 
        }
    } 
public Command getAutonomousCommand() {
    try{
        // Load the path you want to follow using its name in the GUI
        PathPlannerPath path = PathPlannerPath.fromPathFile("Example Path");

        // Create a path following command using AutoBuilder. This will also trigger event markers.
        return AutoBuilder.followPath(path);
    } catch (Exception e) {
        DriverStation.reportError("Big oops: " + e.getMessage(), e.getStackTrace());
        return Commands.none();
    }
  }
    public Command buildAuto() {
        Command auto;
        auto = auto.andThen(AutoBuilder.followPath(startingPosition.getSelected()));
        for (ScoringGroup group : scoringGroups) {
            auto 
        }
        return auto;
    }

    enum StartingPosition { 
        TOP("Starting 1 to ReefR1.path"), 
        MIDDLE("Starting 2 to ReefR2.path"), 
        BOTTOM("Starting 3 to ReefR3.path"); 

        private String pathID;
        StartingPosition(String pathID) {
            this.pathID = pathID;
        }
    }

    enum ReefSide{
        NOCHOICE(""),
        REEFR1(""),
        REEFR2(""),
        REEFR3(""),
        REEFL1(""),
        REEFL2(""),
        REEFL3("");

        private String pathID;
        ReefSide(String pathID) {
            this.pathID = pathID;
        }
    }
    
    enum Level{
        NOCHOICE(""),
        L1(""),
        L2(""),
        L3(""),
        L4("");

        private String pathID = "";
        Level(String pathID) {
            this.pathID = pathID;
        }
    }

    enum Pole{
        NOCHOICE(""),
        LEFTPOLE(""),
        RIGHTPOLE("");

        private String pathID = "";
        Pole(String pathID) {
            this.pathID = pathID;
        }
    }

    enum AlternateDestination {
        NOCHOICE(""),
        LEFTCORAL(""),
        UPCORAL(""),
        DOWNCORAL("");

        private String pathID = "";
        AlternateDestination(String pathID) {
            this.pathID = pathID;
        }
    }

    public class ScoringGroup {
        // add sendable choosers
        private SendableChooser<ReefSide> reefSide;
        private SendableChooser<Level> level; 
        private SendableChooser<Pole> pole; 
        private SendableChooser<AlternateDestination> alternateDestination;

        public ScoringGroup() {
        SmartDashboard.putData("What side are you scoring on", reefSide);
        SmartDashboard.putData("What level are you scoring?", level);
        SmartDashboard.putData("Are you scoring on the right pole or left pole?", pole);

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

        alternateDestination.setDefaultOption("No Choice", AlternateDestination.NOCHOICE);
        alternateDestination.setDefaultOption("Left Coral", AlternateDestination.LEFTCORAL);
        alternateDestination.setDefaultOption("Down Coral", AlternateDestination.DOWNCORAL);
        alternateDestination.setDefaultOption("Up Coral", AlternateDestination.UPCORAL);
        }
    }
}
