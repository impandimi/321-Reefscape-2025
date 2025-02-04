/* (C) Robolancers 2025 */
package frc.robot.subsystems.coralendeffector;

// pidff configuration for the coral end effector; each io implementation will have their own PIDFF
// config
// kP - Proportion constant for the PID config
// kI - Integral constant for the PID config
// kD - Derivative constant for the PID config
// kV - Velocity feedforward constant, proportional to how fast the motor is commanded to run
public record CoralEndEffectorConfig(double kP, double kI, double kD, double kV) {}
