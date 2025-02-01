/* (C) Robolancers 2025 */
package frc.robot.subsystems.algaeIntakePivot;

public record AlgaeIntakePivotConfig(
    double kP, double kI, double kD, double kG) {} // pid/feedforward configuration
// kP = translation
// kI = proportional to integral of error
// kD = slows motion as object reaches target for greater accuracy
// kG = counteracts gravity