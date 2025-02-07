package frc.robot.auto;

import java.util.ArrayList;
import java.util.function.Consumer;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.CoralSuperstructure;
import frc.robot.subsystems.coralendeffector.CoralEndEffector;

public class AutomaticAutonomousMaker3000 {
    
    private SendableChooser<StartingPosition> startingPosition = new SendableChooser<>();
    private SendableChooser<ReefSide> reefSide = new SendableChooser<>();
    private SendableChooser<AlternateDestination> alternateDestination = new SendableChooser<>();

    private ArrayList<ScoringGroup> scoringGroups = new ArrayList<>(); 

    public AutomaticAutonomousMaker3000(CoralEndEffector coralEndEffector, CoralSuperstructure coralSuperstructure) {

        startingPosition.setDefaultOption("Top", StartingPosition.TOP);
        startingPosition.addOption("Middle", StartingPosition.MIDDLE);
        startingPosition.addOption("Bottom", StartingPosition.BOTTOM);

        reefSide.setDefaultOption("No choice", ReefSide.NOCHOICE);
        reefSide.addOption("ReefL1", ReefSide.REEFL1);
        reefSide.addOption("ReefL2", ReefSide.REEFL2);
        reefSide.addOption("ReefL3", ReefSide.REEFL3); 
        reefSide.addOption("ReefR1", ReefSide.REEFR1);
        reefSide.addOption("ReefR2", ReefSide.REEFR2);
        reefSide.addOption("ReefR3", ReefSide.REEFR3);

        alternateDestination.setDefaultOption("No Choice", AlternateDestination.NOCHOICE);
        alternateDestination.addOption("Left Coral 1", AlternateDestination.LEFTCORAL1);
        alternateDestination.addOption("Left Coral 2", AlternateDestination.LEFTCORAL2);
        alternateDestination.addOption("Left Coral 3", AlternateDestination.LEFTCORAL3);
        alternateDestination.addOption("Down Coral", AlternateDestination.DOWNCORAL);
        alternateDestination.addOption("Up Coral", AlternateDestination.UPCORAL);
        
        SmartDashboard.putData("Starting Position", startingPosition);
        SmartDashboard.putData("Reef side", reefSide);
        SmartDashboard.putData("Alternate", alternateDestination);
 
        for (int i = 0; i < 6; i++) {
            scoringGroups.add(new ScoringGroup(i)); 
        }
    } 

    public Command buildAuto(CoralEndEffector coralEndEffector, CoralSuperstructure coralSuperstructure) {
        Command auto = Commands.none();

        auto = auto.andThen(withScoring(getAutonomousCommand(startingPosition.getSelected().pathID 
        + " to " 
        + reefSide.getSelected().pathID), coralSuperstructure))
        .andThen(withIntaking(getAutonomousCommand(reefSide.getSelected().pathID
        + " to "
        + scoringGroups.get(0).alternateDestination.getSelected().pathID), coralEndEffector))
        .andThen(withScoring(getAutonomousCommand(alternateDestination.getSelected().pathID
        + " to "
        + scoringGroups.get(0).reefSide.getSelected().pathID), coralSuperstructure));
        
        for (int i = 0; i < scoringGroups.size() - 1; i++) {

            if (scoringGroups.get(i).alternateDestination.getSelected() == AlternateDestination.NOCHOICE) break; 

            auto = auto.andThen(withIntaking(getAutonomousCommand(scoringGroups.get(i).reefSide.getSelected().pathID 
            + " to " 
            + scoringGroups.get(i).alternateDestination.getSelected().pathID), coralEndEffector))
            .andThen(withScoring(getAutonomousCommand(scoringGroups.get(i).alternateDestination.getSelected().pathID 
            + " to " 
            + scoringGroups.get(i + 1).reefSide.getSelected().pathID), coralSuperstructure));
        }

        return auto;
    }

    public Command withIntaking(Command path, CoralEndEffector coralendeffector) {
        return path; 
        // return path.alongWith(coralendeffector.intakeCoral()).until(coralendeffector::hasCoral);
    }

    public Command withScoring(Command path, CoralSuperstructure coralSuperstructure)  {
        return path; 
        // return path.andThen(coralSuperstructure.outtakeCoral());
    }

    public Command getAutonomousCommand(String pathName) {
        System.out.println(pathName);
        try{
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

    enum ReefSide{
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
        NOCHOICE("Brake"),
        LEFTCORAL1("LeftCoral1"),
        LEFTCORAL2("LeftCoral2"),
        LEFTCORAL3("LeftCoral3"),
        UPCORAL("UpCoral"),
        DOWNCORAL("DownCoral");

        private String pathID = "";
        AlternateDestination(String pathID) {
            this.pathID = pathID;
        }
    }

    public class ScoringGroup {
        // add sendable choosers
        private SendableChooser<ReefSide> reefSide = new SendableChooser<>();
        private SendableChooser<Level> level = new SendableChooser<>(); 
        private SendableChooser<Pole> pole = new SendableChooser<>(); 
        private SendableChooser<AlternateDestination> alternateDestination = new SendableChooser<>();

        public ScoringGroup(int index) {

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
        alternateDestination.addOption("Left Coral 1", AlternateDestination.LEFTCORAL1);
        alternateDestination.addOption("Left Coral 2", AlternateDestination.LEFTCORAL2);
        alternateDestination.addOption("Left Coral 3", AlternateDestination.LEFTCORAL3);
        alternateDestination.addOption("Down Coral", AlternateDestination.DOWNCORAL);
        alternateDestination.addOption("Up Coral", AlternateDestination.UPCORAL);

        
        SmartDashboard.putData("Reef Side" + index, reefSide);
        SmartDashboard.putData("Level" + index, level);
        SmartDashboard.putData("Pole" + index, pole);
        SmartDashboard.putData("Alternate" + index, alternateDestination);
        }
    }
}
