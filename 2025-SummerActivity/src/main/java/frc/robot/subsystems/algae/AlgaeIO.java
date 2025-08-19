package frc.robot.subsystems.algae;

import org.littletonrobotics.junction.AutoLog;

public interface AlgaeIO {
  @AutoLog
  class AlgaeIOInputs {
    public boolean armConnected = false;
    public double armPosition = 0.0; // in rotations
    public double armVelocity = 0.0; // in rotations per second
    public double armAppliedVolts = 0.0; // in volts
    public double armCurrentAmps = 0.0; // in amps

    public boolean rollerConnected = false;
    public double rollerPosition = 0.0; // in rotations
    public double rollerVelocity = 0.0; // in rotations per second
    public double rollerAppliedVolts = 0.0; // in volts
    public double rollerCurrentAmps = 0.0; // in amps
  }

  public default void updateInputs(AlgaeIOInputs inputs) {
    // Default implementation does nothing
  }

  public default void runArm(double speed) {
    // Default implementation does nothing
  }

  public default void stopArm() {
    // Default implementation does nothing
  }

  public default void runRoller(double speed) {
    // Default implementation does nothing
  }

  public default void stopRoller() {
    // Default implementation does nothing
  }
}
