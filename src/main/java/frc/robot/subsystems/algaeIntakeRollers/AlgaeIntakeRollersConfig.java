/* (C) Robolancers 2025 */
package frc.robot.subsystems.algaeIntakeRollers;

public record AlgaeIntakeRollersConfig(
    double kP, double kI, double kD, double kV) {} // pid/feedforward configuration
// kP = translation
// kI = proportional to integral of error
// kD = slows motion as object reaches target for greater accuracy
// kV = velocity