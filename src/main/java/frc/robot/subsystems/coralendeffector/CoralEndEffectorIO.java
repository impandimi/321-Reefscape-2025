/* (C) Robolancers 2025 */
package frc.robot.subsystems.coralendeffector;

public interface CoralEndEffectorIO {
  default void updateInputs(CoralEndEffectorInputs inputs) {}

  default void setVoltage(double voltage) {}
}
