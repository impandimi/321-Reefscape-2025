/* (C) Robolancers 2025 */
package frc.robot.subsystems.algaeIntakerollers;

import edu.wpi.first.epilogue.Logged;

// pid/feedforward configurations
// kP = translation
// kI = proportional to integral of error
// kD = slows motion as object reaches target for greater accuracy
// kV = velocity
@Logged
public record AlgaeIntakeRollersConfig(double kP, double kI, double kD, double kV) {}
