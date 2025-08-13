package frc.robot.subsystems.arm;

import org.littletonrobotics.junction.AutoLog;

public interface ArmIO {
  @AutoLog
  class ArmIOinputs{
    public boolean armConnected = false;
    public double armPosition = 0.0; // in rotations
    public double armVelocity = 0.0; // in rotations per second
    public double armAppliedVolts = 0.0; // in volts
    public double armCurrentAmps = 0.0; // in amps
  }

  public default void updateInputs(ArmIOinputs inputs) {
    // Default implementation does nothing
  }

  public default void positionArm(double pos) {
    // Default implementation does nothing
  }

  public default void stopArm() {
    // Default implementation does nothing
  }
}
