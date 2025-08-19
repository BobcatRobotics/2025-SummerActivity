package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.AutoLog;

public interface ClimberIO {
  @AutoLog
  class ClimberIOInputs {
    public double climberPosition = 0.0; // in rotations
    public double climberVelocity = 0.0; // in rotations per second
    public double climberAppliedVolts = 0.0; // in volts
    public double climberCurrentAmps = 0.0; // in amps
  }

  public default void updateInputs(ClimberIOInputs inputs) {
    // Default implementation does nothing
  }

  public default void runClimber(double speed) {
    // Default implementation does nothing
  }

  public default void stopClimber() {
    // Default implementation does nothing
  }
}
