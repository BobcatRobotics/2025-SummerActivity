package frc.robot.subsystems.algaeRemover;

import org.littletonrobotics.junction.AutoLog;

public interface AlgaeRemoverIO  {

  @AutoLog
  public static class AlgaeRemoverIOInputs {
    /** Whether the motor is connected and responsive. */
    public boolean connected = false;

    /** Position of the motor in radians. */
    public double armPositionRad = 0.0;

    /** Velocity of the motor in radians per second. */
    public double armVelocityRadPerSec = 0.0;

    /** Voltage applied to the motor in volts. */
    public double armAppliedVolts = 0.0;

    /** Current drawn by the motor in amps. */
    public double armCurrentAmps = 0.0;


    /** Position of the motor in radians. */
    public double rollerPositionRad = 0.0;

    /** Velocity of the motor in radians per second. */
    public double rollerVelocityRadPerSec = 0.0;

    /** Voltage applied to the motor in volts. */
    public double rollerAppliedVolts = 0.0;

    /** Current drawn by the motor in amps. */
    public double rollerCurrentAmps = 0.0;


    /** Current state of the motor, as defined by {@link MotorState}. */
    public AlgaeRemoverRollerState rollerState = AlgaeRemoverRollerState.IDLE;
    public AlgaeRemoverArmState armState = AlgaeRemoverArmState.IDLE;
  }

  /**
   * Update the motor input values. This method is called periodically to refresh the telemetry data
   * stored in {@link MotorIOInputs}.
   *
   * @param inputs The container to populate with the current motor telemetry data.
   */
  public default void updateInputs(AlgaeRemoverIOInputs inputs) {}

  public default void runArm(double positionInRotations) {}
  public default void runRoller(double positionInRotations) {}
  ;

  public default void stopArm() {}
  public default void stopRoller(){}
  ;
}