/* (C) Robolancers 2025 */
package frc.robot.subsystems.coralendeffector;

// pidff configuration for the coral end effector; each io implementation will have their own PIDFF
// config
public record CoralEndEffectorConfig(double kP, double kI, double kD, double kV) {}
