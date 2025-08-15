package frc.robot.subsystems.arm;

import org.littletonrobotics.junction.AutoLog;

public interface ArmModuleIO {

  @AutoLog
  public static class ArmModuleIOInputs {
    /** Whether the motor is connected and responsive. */
    public boolean connected = false;

    /** Position of the motor in radians. */
    public double positionRad = 0.0;

    /** Velocity of the motor in radians per second. */
    public double velocityRadPerSec = 0.0;

    /** Voltage applied to the motor in volts. */
    public double appliedVolts = 0.0;

    /** Current drawn by the motor in amps. */
    public double currentAmps = 0.0;

    /** Current state of the motor, as defined by {@link MotorState}. */
    public ArmState state = ArmState.IDLE;
  }

  /**
   * Update the motor input values. This method is called periodically to refresh the telemetry data
   * stored in {@link MotorIOInputs}.
   *
   * @param inputs The container to populate with the current motor telemetry data.
   */
  public default void updateInputs(ArmModuleIOInputs inputs) {}
  public default void runArm(double positionInRotations,double velocity) {}
  public default void runArm(double positionInRotations) {}
  ;

  public default void stopArm() {}
  ;
}
