package frc.robot.subsystems.roller;

import org.littletonrobotics.junction.AutoLog;

public interface RollerIO {
  @AutoLog
  public class RollerIOinputs {
    public boolean rollerConnected = false;
    public double rollerPosition = 0.0; // in rotations
    public double rollerVelocity = 0.0; // in rotations per second
    public double rollerAppliedVolts = 0.0; // in volts
    public double rollerCurrentAmps = 0.0; // in amps
  }

  public default void updateInputs(RollerIOinputs inputs) {
    // Default implementation does nothing
  }

  public default void runRoller(double speed) {
    // Default implementation does nothing
  }

  public default void stopRoller() {
    // Default implementation does nothing
  }
}
