// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.elevator.Elevator;

@Logged
public class RobotContainer {
  
Elevator coolElevator = Elevator.create();
CommandXboxController Controller = new CommandXboxController(0);
  public RobotContainer() {
    configureBindings();
  }

  
  private void configureBindings() {
    Controller.a().whileTrue(coolElevator.voltageHoming());
    // Controller.b().whileTrue(coolElevator.goToStation());
    // Controller.y().
    // Controller.x().
    // Controller.a().
    // Controller.rightBumper().
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }




}
