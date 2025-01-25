/* (C) Robolancers 2025 */
package frc.robot;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionConstants;

@Logged
public class RobotContainer {
  // TODO: add subsystems here
  private Vision vision;

  public RobotContainer() {
    // TODO: init subsystems here
    vision = new Vision(VisionConstants.k427CameraConfig);

    configureBindings();
  }

  private void configureBindings() {}

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
