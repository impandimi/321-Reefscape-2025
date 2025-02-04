/* (C) Robolancers 2025 */
package frc.robot.subsystems.elevatorarm;

import edu.wpi.first.epilogue.Logged;

/**
 * Contains the tuning parameters for the ElevatorArm
 *
 * <p>Parameters: kP - Proportion constant of the Arm PID kI - Integral constant of the Arm PID kD -
 * Derivative constant of the Arm PID kG - The voltage required to keep the arm up when it is
 * horizontal kCoralFF - The ADDITIONAL voltage required to keep the arm with the coral up when it
 * is horizontal
 */
@Logged
public record ElevatorArmConfig(double kP, double kI, double kD, double kG, double kCoralFF) {}
