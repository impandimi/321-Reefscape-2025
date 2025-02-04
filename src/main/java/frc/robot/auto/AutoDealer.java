// package frc.robot.auto;

// import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.Commands;

// public class AutoDealer {
    
//     private SendableChooser<Level> desiredLevel; 

//     public AutoDealer() {
//         desiredLevel.setDefaultOption("No Level", Level.NO_LEVEL);
//         desiredLevel.addOption("L1", Level.L1);


//         SmartDashboard.putData("Desired Level", desiredLevel);
//     }

//     //
//     public Command buildAutoCommand() {
//         Command ret = Commands.none(); 

//         Level wantedLevel = desiredLevel.getSelected();


//         ret = ret.andThen(...); 

//         return ret; 
//     }


//     enum Level {
//         NO_LEVEL(""), 
//         LEFT("Starting 1"), 
//         MIDDLE("Starting 2"), 
//         RIGHT("Starting 3"); 

//         private String pathId; 
        
//         Level(String id) {
//             this.pathId = id; 
//         } 
//     }

//     class ScoringGroup {
//         // add sendable choosers
//     }

// }
