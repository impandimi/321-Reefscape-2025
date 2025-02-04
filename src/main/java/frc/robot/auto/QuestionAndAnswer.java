package frc.robot.auto;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class QuestionAndAnswer {
    
    private SendableChooser<StartingPosition> startingPosition;
    private SendableChooser<Park> park;
    private SendableChooser<ReefSide1> reefSide1;
    private SendableChooser<Level1> level1; 
    private SendableChooser<Pole1> pole1; 
    private SendableChooser<SecondDestination> secondDestination;
    private SendableChooser<ReefSide2> reefSide2;
    private SendableChooser<Level2> level2; 
    private SendableChooser<Pole2> pole2; 
    private SendableChooser<ThirdDestination> thirdDestination;
    private SendableChooser<ReefSide3> reefSide3;
    private SendableChooser<Level3> level3; 
    private SendableChooser<Pole3> pole3;
    private SendableChooser<FourthDestination> fourthDestination;
    private SendableChooser<ReefSide4> reefSide4;
    private SendableChooser<Level4> level4; 
    private SendableChooser<Pole4> pole4; 

    public QuestionAndAnswer() {
        startingPosition.setDefaultOption("Top", StartingPosition.TOP);
        startingPosition.addOption("Middle", StartingPosition.MIDDLE);
        startingPosition.addOption("Bottom", StartingPosition.BOTTOM);
        
        park.setDefaultOption("Yes", Park.YES);
        park.addOption("No", Park.NO);

        reefSide1.setDefaultOption("ReefR1", ReefSide1.REEFR1);
        reefSide1.addOption("ReefR2", ReefSide1.REEFR2);
        reefSide1.addOption("ReefR3", ReefSide1.REEFR3);
        reefSide1.addOption("ReefL1", ReefSide1.REEFL1);
        reefSide1.addOption("ReefL2", ReefSide1.REEFL2);
        reefSide1.addOption("ReefL3", ReefSide1.REEFL3);
        
        level1.setDefaultOption("L1", Level1.L1);
        level1.addOption("L2", Level1.L2);
        level1.addOption("L3", Level1.L3);
        level1.addOption("L4", Level1.L4);

        pole1.setDefaultOption("Right", Pole1.RIGHTPOLE);
        pole1.setDefaultOption("Left", Pole1.LEFTPOLE);

        secondDestination.setDefaultOption("LeftCoral", SecondDestination.LEFTCORAL);
        secondDestination.addOption("UpCoral", SecondDestination.UPCORAL);
        secondDestination.addOption("DownCoral", SecondDestination.DOWNCORAL);
        secondDestination.addOption("SameStation", SecondDestination.SAMESTATION);
        secondDestination.addOption("StopAuto", SecondDestination.STOPAUTO);

        
        reefSide2.setDefaultOption("ReefR1", ReefSide2.REEFR1);
        reefSide2.addOption("ReefR2", ReefSide2.REEFR2);
        reefSide2.addOption("ReefR3", ReefSide2.REEFR3);
        reefSide2.addOption("ReefL1", ReefSide2.REEFL1);
        reefSide2.addOption("ReefL2", ReefSide2.REEFL2);
        reefSide2.addOption("ReefL3", ReefSide2.REEFL3);
        
        level2.setDefaultOption("L1", Level2.L1);
        level2.addOption("L2", Level2.L2);
        level2.addOption("L3", Level2.L3);
        level2.addOption("L4", Level2.L4);

        pole2.setDefaultOption("Right", Pole2.RIGHTPOLE);
        pole2.setDefaultOption("Left", Pole2.LEFTPOLE);
        
        thirdDestination.setDefaultOption("LeftCoral", ThirdDestination.LEFTCORAL);
        thirdDestination.addOption("UpCoral", ThirdDestination.UPCORAL);
        thirdDestination.addOption("DownCoral", ThirdDestination.DOWNCORAL);
        thirdDestination.addOption("SameStation", ThirdDestination.SAMESTATION);
        thirdDestination.addOption("StopAuto", ThirdDestination.STOPAUTO);

        
        reefSide3.setDefaultOption("ReefR1", ReefSide3.REEFR1);
        reefSide3.addOption("ReefR2", ReefSide3.REEFR2);
        reefSide3.addOption("ReefR3", ReefSide3.REEFR3);
        reefSide3.addOption("ReefL1", ReefSide3.REEFL1);
        reefSide3.addOption("ReefL2", ReefSide3.REEFL2);
        reefSide3.addOption("ReefL3", ReefSide3.REEFL3);
        
        level3.setDefaultOption("L1", Level3.L1);
        level3.addOption("L2", Level3.L2);
        level3.addOption("L3", Level3.L3);
        level3.addOption("L4", Level3.L4);

        pole3.setDefaultOption("Right", Pole3.RIGHTPOLE);
        pole3.setDefaultOption("Left", Pole3.LEFTPOLE);

        fourthDestination.setDefaultOption("LeftCoral", FourthDestination.LEFTCORAL);
        fourthDestination.addOption("UpCoral", FourthDestination.UPCORAL);
        fourthDestination.addOption("DownCoral", FourthDestination.DOWNCORAL);
        fourthDestination.addOption("SameStation", FourthDestination.SAMESTATION);
        fourthDestination.addOption("StopAuto", FourthDestination.STOPAUTO);

        
        reefSide4.setDefaultOption("ReefR1", ReefSide4.REEFR1);
        reefSide4.addOption("ReefR2", ReefSide4.REEFR2);
        reefSide4.addOption("ReefR3", ReefSide4.REEFR3);
        reefSide4.addOption("ReefL1", ReefSide4.REEFL1);
        reefSide4.addOption("ReefL2", ReefSide4.REEFL2);
        reefSide4.addOption("ReefL3", ReefSide4.REEFL3);
        
        level4.setDefaultOption("L1", Level4.L1);
        level4.addOption("L2", Level4.L2);
        level4.addOption("L3", Level4.L3);
        level4.addOption("L4", Level4.L4);

        pole4.setDefaultOption("Right", Pole4.RIGHTPOLE);
        pole4.setDefaultOption("Left", Pole4.LEFTPOLE);
        
        SmartDashboard.putData("Starting Position", startingPosition);
        SmartDashboard.putData("Are you Parking?", park);
        SmartDashboard.putData("What side are you scoring on", reefSide1);
        SmartDashboard.putData("What level are you scoring?", level1);
        SmartDashboard.putData("Are you scoring on the right pole or left pole?", pole1);
        SmartDashboard.putData("Where are you going now?", secondDestination);
        SmartDashboard.putData("What side are you scoring on", reefSide2);
        SmartDashboard.putData("What level are you scoring?", level2);
        SmartDashboard.putData("Are you scoring on the right pole or left pole?", pole2);
        SmartDashboard.putData("Where are you going now?", thirdDestination);
        SmartDashboard.putData("What side are you scoring on", reefSide3);
        SmartDashboard.putData("What level are you scoring?", level3);
        SmartDashboard.putData("Are you scoring on the right pole or left pole?", pole3);
        SmartDashboard.putData("Where are you going now?", fourthDestination);
        SmartDashboard.putData("What side are you scoring on", reefSide4);
        SmartDashboard.putData("What level are you scoring?", level4);
        SmartDashboard.putData("Are you scoring on the right pole or left pole?", pole4);
    } 

    enum StartingPosition { 
        TOP, 
        MIDDLE, 
        BOTTOM; 
    }

    enum Park {
        YES,
        NO;
    }
    
    enum ReefSide1{
        REEFR1,
        REEFR2,
        REEFR3,
        REEFL1,
        REEFL2,
        REEFL3;
    }
    
    enum Level1 {
        L1,
        L2,
        L3,
        L4;
    }

    enum Pole1 {
        LEFTPOLE,
        RIGHTPOLE;
    }

    enum SecondDestination{
        LEFTCORAL,
        UPCORAL,
        DOWNCORAL,
        STOPAUTO,
        SAMESTATION;
    }

    enum ReefSide2{
        REEFR1,
        REEFR2,
        REEFR3,
        REEFL1,
        REEFL2,
        REEFL3;
    }

    enum Level2 {
        L1,
        L2,
        L3,
        L4;
    }

    enum Pole2 {
        LEFTPOLE,
        RIGHTPOLE;
    }

    enum ThirdDestination{
        LEFTCORAL,
        UPCORAL,
        DOWNCORAL,
        STOPAUTO,
        SAMESTATION;
    }

    enum ReefSide3{
        REEFR1,
        REEFR2,
        REEFR3,
        REEFL1,
        REEFL2,
        REEFL3;
    }

    enum Level3 {
        L1,
        L2,
        L3,
        L4;
    }

    enum Pole3 {
        LEFTPOLE,
        RIGHTPOLE;
    }

    enum FourthDestination{
        LEFTCORAL,
        UPCORAL,
        DOWNCORAL,
        STOPAUTO,
        SAMESTATION;
    }

    enum ReefSide4{
        REEFR1,
        REEFR2,
        REEFR3,
        REEFL1,
        REEFL2,
        REEFL3;
    }

    enum Level4 {
        L1,
        L2,
        L3,
        L4;
    }

    enum Pole4 {
        LEFTPOLE,
        RIGHTPOLE;
    }
    
    class ScoringGroup {
        // add sendable choosers
    }


}
